/*
 * realTime.c — real-time environment setup for the ADC acquisition thread
 *
 * Calling setup_rt() before the acquisition loop reduces scheduling jitter
 * and cache-miss latency so that DRDY edges are serviced within a few
 * microseconds rather than tens of milliseconds.  Four techniques are used:
 *
 *   1. mlockall()           — pin all process pages into RAM, preventing
 *                             page faults during the hot loop.
 *   2. CPU affinity (core 3)— isolate the acquisition thread to one core
 *                             so the OS scheduler does not migrate it and
 *                             the core's L1/L2 cache stays warm.
 *   3. SCHED_FIFO priority  — preempt any non-RT task; the thread runs
 *                             until it blocks (e.g. on DRDY wait), not
 *                             until a time-slice expires.
 *   4. cpu_dma_latency=0    — write 0 to the PM QoS latency device to
 *                             prevent the kernel from putting the CPU into
 *                             deep C-states between DRDY pulses.  Deep
 *                             C-states add hundreds of microseconds of
 *                             wake-up latency, which would cause DRDY
 *                             edges to be missed at high sample rates.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

void setup_rt(void) {

    printf("Setting up real-time environment...\n");

    /* Lock all current and future memory mappings into RAM.
     * Without this, the kernel may page out code or data between DRDY
     * pulses, causing a page fault — and a multi-millisecond stall — the
     * first time a rarely-used path is executed under load. */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Pin this thread to CPU core 3 (the last core on a 4-core RPi5).
     * Keeping the acquisition loop on a dedicated core avoids cache
     * invalidation from other threads and reduces OS-induced jitter. */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    /* Switch to SCHED_FIFO at priority 90.
     * SCHED_FIFO is a hard real-time policy: the thread preempts all
     * SCHED_OTHER (normal) tasks and runs without a time-slice limit.
     * Priority 90 leaves headroom below 99 for kernel interrupt threads
     * (e.g. SPI DMA completion) which need to run at higher priority. */
    struct sched_param sp = { .sched_priority = 90 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    /* Request PM QoS latency = 0 µs to prevent the CPU from entering
     * C1 or deeper idle states.  Writing 0 (a signed 32-bit zero) to
     * /dev/cpu_dma_latency registers a latency constraint with the
     * kernel power-management subsystem.  The file descriptor must be
     * kept open for the constraint to remain active — intentionally
     * left open here so it persists for the lifetime of the process. */
    int lat_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (lat_fd >= 0) {
        static int32_t zero = 0;
        write(lat_fd, &zero, sizeof(zero));
    }
}
