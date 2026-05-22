#ifndef _SHARED_MEMORY_H
#define _SHARED_MEMORY_H

#include <stdint.h>
#include <stdatomic.h>
#include <sys/mman.h>


// ── Shared memory layout ─────────────────────────────────
#define SHM_NAME     "/ads1263"
#define SHM_SAMPLES  4096    // ring buffer per channel

typedef struct {
    // Pair 0 ring buffer  (producer: head  |  consumer: tail)
    float       pair0[SHM_SAMPLES];
    atomic_uint pair0_head;           // producer write index
    atomic_uint pair0_tail;           // consumer read index
    atomic_uint pair0_lost;           // samples overwritten before consumer read
    atomic_int  current_pair0_raw;

    // Pair 1 ring buffer
    float       pair1[SHM_SAMPLES];
    atomic_uint pair1_head;
    atomic_uint pair1_tail;
    atomic_uint pair1_lost;
    atomic_int  current_pair1_raw;

    // Global status
    atomic_uint  sample_count;
    atomic_uint  overruns;            // SPI read latency violations (>500 µs)
    uint32_t     sample_rate;

    // Runtime commands (Python → C): 0xFF = no pending command
    atomic_uint  cmd_dr;              // DR register byte to apply (0x00–0x0F), or 0xFF
    atomic_uint  cmd_filter;          // filter index 0–4 (SINC1..FIR), or 0xFF
    atomic_uint  cmd_gain;            // GAIN_* register byte (0x00–0x50), or 0xFF
    atomic_uint  cmd_refmux;          // REFMUX_* register byte, or 0xFF
    atomic_uint  cmd_inpmux;          // INPMUX register byte (MUXP<<4|MUXN), or 0xFF
    // Raw register write (Python → C): (addr<<8)|val, 0xFFFFFFFF = nop
    atomic_uint  cmd_raw_wreg;
    // Conversion control (Python → C): 1=START1, 0=STOP1, 0xFF=nop
    atomic_uint  cmd_conv;
    // Register read-back (C → Python): updated after init and each write
    atomic_uint  rb_mode0;
    atomic_uint  rb_mode1;
    atomic_uint  rb_mode2;
    atomic_uint  rb_refmux;
    atomic_uint  rb_inpmux;
} AdsShm;

AdsShm *shm_create(void);

#endif // _SHARED_MEMORY_H