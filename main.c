/*
 * main.c — ADS1263 real-time acquisition loop
 *
 * Architecture overview
 * ─────────────────────
 * This process runs on the Raspberry Pi 5 and owns the SPI bus and the
 * DRDY GPIO line.  It initialises the ADS1263 32-bit delta-sigma ADC,
 * then spins in a tight loop:
 *
 *   1. Poll for pending configuration commands written by the Python UI
 *      into POSIX shared memory (/ads1263).
 *   2. Block on the DRDY falling edge (new conversion result ready).
 *   3. Read the 32-bit raw result via SPI (RDATA1 command, 5 bytes).
 *   4. Convert to volts and push the sample into a lock-free SPSC ring
 *      buffer inside the shared memory region.
 *
 * The Python UI reads samples from the ring buffer asynchronously and
 * sends configuration changes back through the same shared memory using
 * atomic "command" fields (sentinel 0xFF / 0xFFFFFFFF = no pending cmd).
 *
 * Hardware connections (BCM pin numbering)
 * ─────────────────────────────────────────
 *   GPIO 18 (BCM) → ADS1263 RESET
 *   GPIO 27 (BCM) → ADS1263 DRDY   (falling edge = conversion complete)
 *   SPI0           → ADS1263 SCLK / MOSI / MISO / CS0
 */

#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <gpiod.h>


#include "realTime.h"       // setup_rt(): elevate SCHED_FIFO priority
#include "ads1263.h"        // ADC driver + register/command definitions
#include "sharedMemory.h"   // AdsShm layout + shm_create()

// ── Hardware config ──────────────────────────────────────

/* libgpiod v2 GPIO chip device for RPi5 (gpiochip4 = main GPIO bank) */
#define GPIO_CHIP     "/dev/gpiochip4"

/* BCM pin number for the ADS1263 DRDY signal (active-low, pulsed low when
 * a new conversion result is available).  Connected to GPIO BCM 27. */
#define PIN_DRDY                        27

/* If a single SPI read + volt conversion takes longer than this many
 * microseconds the sample is still stored but an overrun counter is
 * incremented so the Python UI can detect latency problems. */
#define OVERRUN_THRESHOLD_US            2000

/* Print a one-line statistics summary to stdout every N samples. */
#define PRINT_STATS_INTERVAL_SAMPLES    2000

// ── Globals ──────────────────────────────────────────────

/* Set to 0 by the signal handler to break the acquisition loop cleanly. */
static volatile int running = 1;

/* libgpiod v2 handles — opened once at startup, released at shutdown. */
static struct gpiod_chip         *pChip     = NULL;  // GPIO chip handle
static struct gpiod_line_request *drdy_req  = NULL;  // DRDY line request

/* Pointer to the POSIX shared memory region mapped into this process.
 * The Python UI maps the same region to exchange samples and commands. */
static AdsShm                    *shm       = NULL;

/** LOCAL FUNCTION PROTOTYPES */

/* Block until DRDY falls or timeout_ms elapses.
 * Returns >0 on event, 0 on timeout, -1 on error (matches gpiod semantics). */
static int8_t wait_drdy(int timeout_ms);

/* SIGINT / SIGTERM handler — sets running=0 to exit the acquisition loop. */
static void sig_handler(int s);

// ── Main acquisition loop ─────────────────────────────────
int main(void){

    int64_t samplesCnt = 0;   // total conversions completed this session
    int64_t timeoutCnt = 0;   // DRDY timeouts (ADC not responding in time)

    /* libgpiod edge-event buffer — we only need one event slot because we
     * consume each DRDY pulse immediately before waiting for the next. */
    struct gpiod_edge_event_buffer *evbuf = gpiod_edge_event_buffer_new(1);

    /* Flush stdout after every write so Python can read progress lines even
     * when stdout is piped (subprocess.Popen with stdout=PIPE). */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("ADS1263 SPI Oscilloscope\n");

    /* Install signal handlers so Ctrl-C and systemd stop both trigger a
     * clean shutdown (ADC stopped, SPI closed, SHM unlinked). */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // ── GPIO setup ───────────────────────────────────────────
    pChip = gpiod_chip_open(GPIO_CHIP);
    if (!pChip) { perror("gpiod_chip_open"); return 1; }

    /* Request the DRDY line as an input with falling-edge event detection.
     * libgpiod v2 API requires building a settings/config/request chain. */
    {
        struct gpiod_line_settings  *s  = gpiod_line_settings_new();
        struct gpiod_line_config    *lc = gpiod_line_config_new();
        struct gpiod_request_config *rc = gpiod_request_config_new();
        unsigned int offset = PIN_DRDY;
        gpiod_line_settings_set_direction(s, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(s, GPIOD_LINE_EDGE_FALLING);
        gpiod_line_config_add_line_settings(lc, &offset, 1, s);
        gpiod_request_config_set_consumer(rc, "adc_oscilloscope");
        drdy_req = gpiod_chip_request_lines(pChip, rc, lc);
        gpiod_line_settings_free(s);
        gpiod_line_config_free(lc);
        gpiod_request_config_free(rc);
    }
    if (!drdy_req) { perror("request DRDY"); return 1; }

    // ── ADC init ─────────────────────────────────────────────
    /* Configure ADS1263 registers via spidev: SPI mode 1, 1 MHz, sets
     * gain/DR/filter/refmux/inpmux from the compile-time defaults in
     * ads1263.h, then issues START1 to begin continuous conversions. */
    if (ads1263_init(pChip)) { perror("ads1263_init"); return 1; }

    // ── Shared memory ────────────────────────────────────────
    /* Create (or re-create) the POSIX shared memory segment /ads1263 and
     * map it.  The Python UI opens the same name with posix_ipc. */
    shm = shm_create();
    if (!shm) { perror("shm_create"); return 1; }

    /* Elevate this thread to SCHED_FIFO real-time priority so that the
     * kernel wakes us immediately when DRDY fires, minimising jitter. */
    setup_rt();

    /* Publish the compile-time sample rate so the Python UI can display it
     * immediately without waiting for a DR change command. */
    shm->sample_rate = ADC_DR_SPS;
    int drdy_timeout_ms = ADC_DRDY_TIMEOUT_MS;  // updated when DR changes

    /* Initialise SHM command sentinels to "no pending command" and populate
     * the register read-back fields so the Python UI shows the actual
     * register contents immediately on connect. */
    atomic_store_explicit(&shm->cmd_raw_wreg, 0xFFFFFFFF, memory_order_relaxed);
    atomic_store_explicit(&shm->cmd_conv,     0xFF,        memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode0,  ads1263_read_reg(REG_MODE0),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode1,  ads1263_read_reg(REG_MODE1),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode2,  ads1263_read_reg(REG_MODE2),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
    atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);

    printf("ADS1263 SPI Oscilloscope Acquisition Started! (%u SPS)\n", ADC_DR_SPS);

    // ── Acquisition loop ──────────────────────────────────────
    while (running) {

        /* ── Process pending commands from Python UI ──────────────
         * Each cmd_* field is polled once per DRDY cycle (~1200 times/s
         * at 1200 SPS).  The Python side writes a value != sentinel to
         * request a change; we apply it and restore the sentinel.
         * Processing order mirrors register dependency: DR and GAIN both
         * write MODE2, so they are handled separately. */

        /* Data rate change: update MODE2[3:0] and recalculate the DRDY
         * timeout for the new conversion period. */
        unsigned cmd = atomic_load_explicit(&shm->cmd_dr, memory_order_relaxed);
        if (cmd != 0xFF) {
            ads1263_set_dr((uint8_t)cmd);
            drdy_timeout_ms   = ADS1263_DR_TIMEOUT_MS[cmd & 0x0F];
            shm->sample_rate  = ADS1263_DR_SPS[cmd & 0x0F];
            atomic_store_explicit(&shm->cmd_dr, 0xFF, memory_order_relaxed);
            /* Refresh MODE2 read-back so the Python register panel updates. */
            atomic_store_explicit(&shm->rb_mode2, ads1263_read_reg(REG_MODE2), memory_order_relaxed);
            printf("DR changed to 0x%02x (%u SPS)\n", cmd, shm->sample_rate);
        }

        /* Digital filter selection: updates MODE1[7:5] (SINC1/2/3/4 or FIR).
         * FIR is only valid at DR ≤ 60 SPS; enforced by ads1263_set_filter(). */
        unsigned cmd_f = atomic_load_explicit(&shm->cmd_filter, memory_order_relaxed);
        if (cmd_f != 0xFF) {
            ads1263_set_filter((uint8_t)cmd_f);
            atomic_store_explicit(&shm->cmd_filter, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode1, ads1263_read_reg(REG_MODE1), memory_order_relaxed);
            printf("Filter changed to %s\n", ADS1263_FILTER_NAME[cmd_f & 0x07]);
        }

        /* PGA gain change: updates MODE2[7:4].  Also updates the rt_gain
         * variable in ads1263.c so raw_to_volts() uses the correct factor. */
        unsigned cmd_g = atomic_load_explicit(&shm->cmd_gain, memory_order_relaxed);
        if (cmd_g != 0xFF) {
            ads1263_set_gain((uint8_t)cmd_g);
            atomic_store_explicit(&shm->cmd_gain, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode2, ads1263_read_reg(REG_MODE2), memory_order_relaxed);
            printf("Gain changed to %s\n", ADS1263_GAIN_NAME[(cmd_g & 0xF0) >> 4]);
        }

        /* Reference mux: selects the voltage reference source for the ADC
         * modulator (internal 2.5 V, AIN0/1, AIN2/3, or AVDD/AVSS).
         * Also updates rt_vref so raw_to_volts() reflects the new Vref. */
        unsigned cmd_r = atomic_load_explicit(&shm->cmd_refmux, memory_order_relaxed);
        if (cmd_r != 0xFF) {
            ads1263_set_refmux((uint8_t)cmd_r);
            atomic_store_explicit(&shm->cmd_refmux, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
            printf("REFMUX changed to 0x%02x\n", cmd_r);
        }

        /* Input mux: selects the positive and negative analog input channels
         * for differential measurement (MUXP[7:4] | MUXN[3:0]). */
        unsigned cmd_m = atomic_load_explicit(&shm->cmd_inpmux, memory_order_relaxed);
        if (cmd_m != 0xFF) {
            ads1263_set_inpmux((uint8_t)cmd_m);
            atomic_store_explicit(&shm->cmd_inpmux, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);
            printf("INPMUX changed to MUXP=AIN%u MUXN=AIN%u\n",
                   (cmd_m >> 4) & 0xF, cmd_m & 0xF);
        }

        /* Conversion start/stop: 1 = issue START1, 0 = issue STOP1.
         * Useful for pausing conversions while reconfiguring registers
         * without using the individual set_* helpers. */
        unsigned cmd_cv = atomic_load_explicit(&shm->cmd_conv, memory_order_relaxed);
        if (cmd_cv != 0xFF) {
            if (cmd_cv)
                ads1263_conv_start();
            else
                ads1263_conv_stop();
            atomic_store_explicit(&shm->cmd_conv, 0xFF, memory_order_relaxed);
            printf("Conversion %s\n", cmd_cv ? "started" : "stopped");
        }

        /* Raw register write: Python packs (addr << 8) | val into a single
         * uint32.  Allows writing any ADS1263 register directly from the
         * "ADC Register" panel in the UI.  All five read-back fields are
         * refreshed afterwards because an arbitrary write could affect any. */
        unsigned cmd_rw = atomic_load_explicit(&shm->cmd_raw_wreg, memory_order_relaxed);
        if (cmd_rw != 0xFFFFFFFF) {
            uint8_t rw_addr = (cmd_rw >> 8) & 0xFF;
            uint8_t rw_val  = cmd_rw & 0xFF;
            ads1263_write_reg_raw(rw_addr, rw_val);
            atomic_store_explicit(&shm->cmd_raw_wreg, 0xFFFFFFFF, memory_order_relaxed);
            /* Refresh all five read-backs since a raw write may affect any register */
            atomic_store_explicit(&shm->rb_mode0,  ads1263_read_reg(REG_MODE0),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode1,  ads1263_read_reg(REG_MODE1),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode2,  ads1263_read_reg(REG_MODE2),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
            atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);
            printf("RAW WREG 0x%02x = 0x%02x\n", rw_addr, rw_val);
        }

        // ── Wait for DRDY ─────────────────────────────────────
        /* Block here until the ADS1263 pulls DRDY low, signalling that a
         * new 32-bit conversion result is latched in its output register.
         * Timeout is calibrated to ~2× the conversion period (see ads1263.h). */
        int ret = wait_drdy(drdy_timeout_ms);
        if (ret <= 0) {
            /* ret == 0 : DRDY did not fire within timeout_ms — the ADC may
             * have been stopped (cmd_conv=0) or is in reset.  Not fatal.
             * ret == -1 : gpiod error — also non-fatal, just count it. */
            timeoutCnt++;
            fprintf(stderr, "DRDY timeout: %ld\n", timeoutCnt);
            continue;
        }

        /* Consume the edge event from the kernel buffer to re-arm the wait.
         * Without this call the next wait_drdy() would return immediately
         * on the stale event already in the buffer. */
        gpiod_line_request_read_edge_events(drdy_req, evbuf, 1);

        /* Timestamp the start of the SPI read so we can detect overruns
         * (cases where the SPI transaction takes too long and risks missing
         * the next DRDY pulse). */
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        // ── Read conversion result ────────────────────────────
        /* Send RDATA1 command over SPI and receive 5 bytes:
         *   byte 0: status register snapshot
         *   bytes 1-4: 32-bit signed result, MSB first
         * raw_to_volts() applies: result = raw * Vref / (gain * 2^31) */
        int32_t raw = ads1263_read();
        float volts = raw_to_volts(raw);

        // ── Push sample into SPSC ring buffer ─────────────────
        /* Lock-free single-producer single-consumer ring buffer.
         * Producer (this loop) advances pair0_head with release semantics
         * so the consumer (Python) sees the written value after the index. */
        unsigned h = atomic_load_explicit(&shm->pair0_head, memory_order_relaxed);
        shm->pair0[h] = volts;
        unsigned next = (h + 1) % SHM_SAMPLES;
        atomic_store_explicit(&shm->pair0_head, next, memory_order_release);

        /* Overwrite detection: if the consumer's tail equals the new head,
         * the consumer has not read the oldest slot yet and we are about to
         * overwrite it.  Advance tail and increment the lost counter. */
        unsigned tail = atomic_load_explicit(&shm->pair0_tail, memory_order_acquire);
        if (next == tail) {
            atomic_fetch_add_explicit(&shm->pair0_lost, 1, memory_order_relaxed);
            atomic_store_explicit(&shm->pair0_tail, (tail + 1) % SHM_SAMPLES, memory_order_relaxed);
        }

        // ── Overrun detection ─────────────────────────────────
        /* Measure how long the SPI read took.  If it exceeds the threshold
         * the system is too slow for the configured sample rate and the next
         * DRDY pulse may already be pending in the kernel buffer, causing
         * the ring buffer to fill faster than the consumer can drain it. */
        clock_gettime(CLOCK_MONOTONIC, &t1);
        long elapsed_us = (t1.tv_sec  - t0.tv_sec)  * 1000000L
                        + (t1.tv_nsec - t0.tv_nsec) / 1000L;

        if (elapsed_us > OVERRUN_THRESHOLD_US) {
            atomic_fetch_add(&shm->overruns, 1);
            fprintf(stderr, "overrun %ld us (total: %u)\n",
                    elapsed_us, atomic_load(&shm->overruns));
        }

        /* Update global counters visible to the Python UI status bar. */
        samplesCnt++;
        atomic_store(&shm->sample_count, (unsigned int)samplesCnt);
        atomic_store(&shm->current_pair0_raw, raw);  // latest raw value (no ring)

        /* Periodic statistics to stdout for debugging. */
        if (samplesCnt % PRINT_STATS_INTERVAL_SAMPLES == 0) {
            printf("Samples: %u  timeouts: %ld  overruns: %u  lost: %u\n",
                   atomic_load(&shm->sample_count),
                   (long)timeoutCnt,
                   atomic_load(&shm->overruns),
                   atomic_load(&shm->pair0_lost));
            fflush(stdout);
        }

    } // end acquisition loop

    // ── Clean shutdown ────────────────────────────────────────
    printf("Acquisition completed — %ld samples\n", (long)samplesCnt);

    gpiod_edge_event_buffer_free(evbuf);
    gpiod_line_request_release(drdy_req);
    ads1263_deinit();               // STOP1 command, SPI fd closed, RESET GPIO released
    gpiod_chip_close(pChip);

    /* Unmap and unlink the shared memory so the next run starts clean.
     * The Python UI detects the unlink as a disconnect and shows "Stopped". */
    if (shm) {
        munmap(shm, sizeof(AdsShm));
        shm_unlink(SHM_NAME);
    }

    return 0;
}

/** LOCAL FUNCTION IMPLEMENTATION */

// ── Wait for DRDY falling edge with timeout ───────────────
static int8_t wait_drdy(int timeout_ms) {
    int8_t res;
    /* gpiod_line_request_wait_edge_events() takes timeout in nanoseconds. */
    int64_t timeout_ns = (int64_t)timeout_ms * 1000000LL;
    res = gpiod_line_request_wait_edge_events(drdy_req, timeout_ns);
    return res;  // >0 = event ready, 0 = timeout, -1 = error
}

// ── SIGINT / SIGTERM handler ──────────────────────────────
static void sig_handler(int s) {
    (void)s;  // suppress unused-parameter warning
    printf("\nSignal received: %d, exiting...\n", s);
    running = 0;  // causes the acquisition loop to exit on next iteration
}
