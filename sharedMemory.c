#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "sharedMemory.h"

// ── Shared memory setup ───────────────────────────────────
AdsShm *shm_create(void)
{
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return NULL; }

    if (ftruncate(fd, sizeof(AdsShm)) < 0) {
        perror("ftruncate"); close(fd); return NULL;
    }

    AdsShm *p = mmap(NULL, sizeof(AdsShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) { perror("mmap"); return NULL; }

    atomic_store(&p->pair0_head,         0);
    atomic_store(&p->pair0_tail,         0);
    atomic_store(&p->pair0_lost,         0);
    atomic_store(&p->current_pair0_raw,  0);
    atomic_store(&p->pair1_head,         0);
    atomic_store(&p->pair1_tail,         0);
    atomic_store(&p->pair1_lost,         0);
    atomic_store(&p->current_pair1_raw,  0);
    atomic_store(&p->sample_count,       0);
    atomic_store(&p->overruns,           0);
    
    p->sample_rate = 0;
    atomic_store(&p->cmd_dr,     0xFF);
    atomic_store(&p->cmd_filter, 0xFF);
    atomic_store(&p->cmd_gain,   0xFF);
    atomic_store(&p->cmd_refmux, 0xFF);
    atomic_store(&p->cmd_inpmux, 0xFF);

    return p;
}