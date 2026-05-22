/*
 * sharedMemory.c — POSIX shared memory creation and initialisation
 *
 * shm_create() opens (or re-creates) the /ads1263 shared memory object,
 * sizes it to hold one AdsShm struct, maps it into this process, and
 * zeroes / initialises every field so the Python UI sees consistent state
 * from the moment it opens the same name with posix_ipc.
 *
 * Ownership model
 * ───────────────
 * This process (the C acquisition daemon) is the sole *producer* and
 * *owner* of the shared memory:
 *   - It calls shm_open(O_CREAT) — creating the object if absent.
 *   - It calls shm_unlink() at shutdown to remove the name from the
 *     filesystem so the next run starts with a clean slate.
 * The Python UI is a *consumer*: it opens the same name read-write
 * (to write cmd_* fields) but never creates or unlinks it.
 *
 * Command sentinel convention
 * ───────────────────────────
 * All cmd_* fields are initialised to 0xFF (8-bit sentinel) here.
 * The acquisition loop in main.c treats 0xFF as "no pending command"
 * and skips processing.  The Python UI writes a real value to request
 * a change; main.c applies it and restores 0xFF.
 * cmd_raw_wreg uses 0xFFFFFFFF (32-bit) — initialised in main.c after
 * the ADC is ready, so it is not set here.
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "sharedMemory.h"

AdsShm *shm_create(void)
{
    /* Open (or create) the POSIX shared memory object.
     * O_CREAT | O_RDWR: create if absent, open read-write.
     * 0666: world-readable/writable so the Python UI (which may run as a
     * different user) can map it without privilege escalation. */
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return NULL; }

    /* Set the object size to exactly one AdsShm struct.
     * A newly created shm object has size 0; ftruncate() extends it and
     * zero-fills the new pages — providing a safe default for any field
     * we do not explicitly initialise below. */
    if (ftruncate(fd, sizeof(AdsShm)) < 0) {
        perror("ftruncate"); close(fd); return NULL;
    }

    /* Map the object into this process's address space.
     * MAP_SHARED: writes are visible to all other mappings of the same fd.
     * The fd is closed immediately after mmap(); the mapping stays valid
     * until munmap() is called — the kernel keeps a reference count. */
    AdsShm *p = mmap(NULL, sizeof(AdsShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) { perror("mmap"); return NULL; }

    /* ── Ring buffer state ───────────────────────────────────
     * Both pair0 and pair1 ring buffers start empty (head == tail == 0).
     * lost counts samples overwritten before the consumer could read them;
     * current_*_raw holds the latest raw value outside the ring (no index). */
    atomic_store(&p->pair0_head,        0);
    atomic_store(&p->pair0_tail,        0);
    atomic_store(&p->pair0_lost,        0);
    atomic_store(&p->current_pair0_raw, 0);
    atomic_store(&p->pair1_head,        0);
    atomic_store(&p->pair1_tail,        0);
    atomic_store(&p->pair1_lost,        0);
    atomic_store(&p->current_pair1_raw, 0);

    /* ── Global counters ─────────────────────────────────────
     * sample_count: total conversions since start (displayed in status bar).
     * overruns: SPI reads that took longer than OVERRUN_THRESHOLD_US. */
    atomic_store(&p->sample_count, 0);
    atomic_store(&p->overruns,     0);

    /* sample_rate is written by main.c from ADC_DR_SPS after ADC init;
     * set to 0 here so the Python UI can detect "not yet ready". */
    p->sample_rate = 0;

    /* ── Command sentinels ───────────────────────────────────
     * 0xFF = no pending command for all 8-bit cmd_* fields.
     * The Python UI writes a real value to request a change.
     * cmd_raw_wreg (32-bit, sentinel 0xFFFFFFFF) and cmd_conv are
     * initialised in main.c after the ADC and register read-backs
     * are ready, so they are intentionally omitted here. */
    atomic_store(&p->cmd_dr,     0xFF);
    atomic_store(&p->cmd_filter, 0xFF);
    atomic_store(&p->cmd_gain,   0xFF);
    atomic_store(&p->cmd_refmux, 0xFF);
    atomic_store(&p->cmd_inpmux, 0xFF);

    return p;
}