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


#include "realTime.h"
#include "ads1263.h"
#include "sharedMemory.h"

// ── Hardware config ──────────────────────────────────────
#define GPIO_CHIP     "/dev/gpiochip4"

#define PIN_DRDY                        27      // GPIO BCM17 — connect to DRDY

#define OVERRUN_THRESHOLD_US            2000    // warn if SPI read exceeds this

#define PRINT_STATS_INTERVAL_SAMPLES    2000

// ── Globals ──────────────────────────────────────────────
static volatile int running = 1;

static struct gpiod_chip         *pChip     = NULL;
static struct gpiod_line_request *drdy_req  = NULL;
static AdsShm                    *shm       = NULL;

/** LOCAL FUNCTION PROTOTYPES */
static int8_t wait_drdy(int timeout_ms);
static void sig_handler(int s);

// ── Main acquisition loop ─────────────────────────────────
int main(void){

    int64_t samplesCnt = 0;
    int64_t timeoutCnt = 0;

    struct gpiod_edge_event_buffer *evbuf = gpiod_edge_event_buffer_new(1);
    
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("ADS1263 SPI Oscilloscope\n");

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // GPIO
    pChip = gpiod_chip_open(GPIO_CHIP);
    if (!pChip) { perror("gpiod_chip_open"); return 1; }

    /* libgpiod v2: request each line via settings + config objects */
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

    // ADS1263 — configure registers via spidev, then hand off SPI to DMA
    if (ads1263_init(pChip)) { perror("ads1263_init"); return 1; }

    // Shared memory
    shm = shm_create();
    if (!shm) { perror("shm_create"); return 1; }

    setup_rt();

    
    shm->sample_rate = ADC_DR_SPS;
    int drdy_timeout_ms = ADC_DRDY_TIMEOUT_MS;

    // Initialise SHM command sentinels and populate register read-backs
    atomic_store_explicit(&shm->cmd_raw_wreg, 0xFFFFFFFF, memory_order_relaxed);
    atomic_store_explicit(&shm->cmd_conv,     0xFF,        memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode0,  ads1263_read_reg(REG_MODE0),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode1,  ads1263_read_reg(REG_MODE1),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_mode2,  ads1263_read_reg(REG_MODE2),  memory_order_relaxed);
    atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
    atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);

    printf("ADS1263 SPI Oscilloscope Acquisition Started! (%u SPS)\n", ADC_DR_SPS);

    while (running) {
        // Apply pending commands from Python UI
        unsigned cmd = atomic_load_explicit(&shm->cmd_dr, memory_order_relaxed);
        if (cmd != 0xFF) {
            ads1263_set_dr((uint8_t)cmd);
            drdy_timeout_ms   = ADS1263_DR_TIMEOUT_MS[cmd & 0x0F];
            shm->sample_rate  = ADS1263_DR_SPS[cmd & 0x0F];
            atomic_store_explicit(&shm->cmd_dr, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode2, ads1263_read_reg(REG_MODE2), memory_order_relaxed);
            printf("DR changed to 0x%02x (%u SPS)\n", cmd, shm->sample_rate);
        }
        unsigned cmd_f = atomic_load_explicit(&shm->cmd_filter, memory_order_relaxed);
        if (cmd_f != 0xFF) {
            ads1263_set_filter((uint8_t)cmd_f);
            atomic_store_explicit(&shm->cmd_filter, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode1, ads1263_read_reg(REG_MODE1), memory_order_relaxed);
            printf("Filter changed to %s\n", ADS1263_FILTER_NAME[cmd_f & 0x07]);
        }
        unsigned cmd_g = atomic_load_explicit(&shm->cmd_gain, memory_order_relaxed);
        if (cmd_g != 0xFF) {
            ads1263_set_gain((uint8_t)cmd_g);
            atomic_store_explicit(&shm->cmd_gain, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode2, ads1263_read_reg(REG_MODE2), memory_order_relaxed);
            printf("Gain changed to %s\n", ADS1263_GAIN_NAME[(cmd_g & 0xF0) >> 4]);
        }
        unsigned cmd_r = atomic_load_explicit(&shm->cmd_refmux, memory_order_relaxed);
        if (cmd_r != 0xFF) {
            ads1263_set_refmux((uint8_t)cmd_r);
            atomic_store_explicit(&shm->cmd_refmux, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
            printf("REFMUX changed to 0x%02x\n", cmd_r);
        }
        unsigned cmd_m = atomic_load_explicit(&shm->cmd_inpmux, memory_order_relaxed);
        if (cmd_m != 0xFF) {
            ads1263_set_inpmux((uint8_t)cmd_m);
            atomic_store_explicit(&shm->cmd_inpmux, 0xFF, memory_order_relaxed);
            atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);
            printf("INPMUX changed to MUXP=AIN%u MUXN=AIN%u\n",
                   (cmd_m >> 4) & 0xF, cmd_m & 0xF);
        }
        unsigned cmd_cv = atomic_load_explicit(&shm->cmd_conv, memory_order_relaxed);
        if (cmd_cv != 0xFF) {
            if (cmd_cv)
                ads1263_conv_start();
            else
                ads1263_conv_stop();
            atomic_store_explicit(&shm->cmd_conv, 0xFF, memory_order_relaxed);
            printf("Conversion %s\n", cmd_cv ? "started" : "stopped");
        }
        unsigned cmd_rw = atomic_load_explicit(&shm->cmd_raw_wreg, memory_order_relaxed);
        if (cmd_rw != 0xFFFFFFFF) {
            uint8_t rw_addr = (cmd_rw >> 8) & 0xFF;
            uint8_t rw_val  = cmd_rw & 0xFF;
            ads1263_write_reg_raw(rw_addr, rw_val);
            atomic_store_explicit(&shm->cmd_raw_wreg, 0xFFFFFFFF, memory_order_relaxed);
            // Refresh all five read-backs since a raw write may affect any register
            atomic_store_explicit(&shm->rb_mode0,  ads1263_read_reg(REG_MODE0),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode1,  ads1263_read_reg(REG_MODE1),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_mode2,  ads1263_read_reg(REG_MODE2),  memory_order_relaxed);
            atomic_store_explicit(&shm->rb_refmux, ads1263_read_reg(REG_REFMUX), memory_order_relaxed);
            atomic_store_explicit(&shm->rb_inpmux, ads1263_read_reg(REG_INPMUX), memory_order_relaxed);
            printf("RAW WREG 0x%02x = 0x%02x\n", rw_addr, rw_val);
        }

        int ret = wait_drdy(drdy_timeout_ms);
        if (ret <= 0) {
            /* Error (-1), or timeout (0) */
            timeoutCnt++;
            fprintf(stderr, "DRDY timeout: %ld\n", timeoutCnt);
            continue;
        }
        // Consume the DRDY event
        gpiod_line_request_read_edge_events(drdy_req, evbuf, 1);
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        // Read result
        int32_t raw = ads1263_read();
        float volts = raw_to_volts(raw);
        // Push to pair0 ring buffer; detect overwrite if consumer is behind
        unsigned h = atomic_load_explicit(&shm->pair0_head, memory_order_relaxed);
        shm->pair0[h] = volts;
        unsigned next = (h + 1) % SHM_SAMPLES;
        atomic_store_explicit(&shm->pair0_head, next, memory_order_release);

        unsigned tail = atomic_load_explicit(&shm->pair0_tail, memory_order_acquire);
        if (next == tail) {
            // Consumer hasn't read slot `tail` yet — overwrite it and push tail forward
            atomic_fetch_add_explicit(&shm->pair0_lost, 1, memory_order_relaxed);
            atomic_store_explicit(&shm->pair0_tail, (tail + 1) % SHM_SAMPLES, memory_order_relaxed);
        }


        clock_gettime(CLOCK_MONOTONIC, &t1);
        long elapsed_us = (t1.tv_sec  - t0.tv_sec)  * 1000000L
                        + (t1.tv_nsec - t0.tv_nsec) / 1000L;

        if (elapsed_us > OVERRUN_THRESHOLD_US) {
            atomic_fetch_add(&shm->overruns, 1);
            fprintf(stderr, "overrun %ld us (total: %u)\n",
                    elapsed_us, atomic_load(&shm->overruns));
        }

        samplesCnt++;
        atomic_store(&shm->sample_count, (unsigned int)samplesCnt);
        atomic_store(&shm->current_pair0_raw, raw);

        // Print stats every 2000 samples
        if (samplesCnt % PRINT_STATS_INTERVAL_SAMPLES == 0) {
            printf("Samples: %u  timeouts: %ld  overruns: %u  lost: %u\n",
                   atomic_load(&shm->sample_count),
                   (long)timeoutCnt,
                   atomic_load(&shm->overruns),
                   atomic_load(&shm->pair0_lost));
            fflush(stdout);
        }

    }

    printf("Acquisition completed — %ld samples\n", (long)samplesCnt);

    gpiod_edge_event_buffer_free(evbuf);
    gpiod_line_request_release(drdy_req);
    ads1263_deinit();
    gpiod_chip_close(pChip);

    if (shm) {
        munmap(shm, sizeof(AdsShm));
        shm_unlink(SHM_NAME);
    }

    return 0;
}

/** LOCAL FUNCTION IMPLEMENTATION */

// ── Wait for DRDY with timeout ────────────────────────────
static int8_t wait_drdy(int timeout_ms) {
    int8_t res;
    int64_t timeout_ns = (int64_t)timeout_ms * 1000000LL;
    res = gpiod_line_request_wait_edge_events(drdy_req, timeout_ns);
    return res;
}

// ── Signal handler ───────────────────────────────────────
static void sig_handler(int s) {
    (void)s;
    printf("\nSignal received: %d, exiting...\n", s);
    running = 0;
}

