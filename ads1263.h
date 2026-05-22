#ifndef ADS1263_H
#define ADS1263_H

#pragma once
#include <stdint.h>
#include <stdatomic.h>
#include <gpiod.h>

// ── Active data rate — uncomment exactly one ─────────────
//#define ADC_DR  DR_2P5
//#define ADC_DR  DR_5
//#define ADC_DR  DR_10
//#define ADC_DR  DR_16P6
//#define ADC_DR  DR_20
//#define ADC_DR  DR_50
//#define ADC_DR  DR_60
//#define ADC_DR  DR_100
//#define ADC_DR  DR_400
#define ADC_DR  DR_1200
//#define ADC_DR  DR_2400
//#define ADC_DR  DR_4800
//#define ADC_DR  DR_7200
//#define ADC_DR  DR_14400
//#define ADC_DR  DR_19200
//#define ADC_DR  DR_38400

#define ADC_GAIN GAIN_32

#define ADC_REF_MUX REFMUX_AIN2_AIN3
//#define ADC_REF_MUX REFMUX_INTERNAL 

// ── ADS1263 Commands ─────────────────────────────────────
#define ADS1263_CMD_RESET    0x06
#define ADS1263_CMD_START1   0x08
#define ADS1263_CMD_STOP1    0x0A
#define ADS1263_CMD_RDATA1   0x12
#define ADS1263_CMD_RREG     0x20
#define ADS1263_CMD_WREG     0x40

// ── ADS1263 Registers ────────────────────────────────────
#define REG_ID       0x00
#define REG_POWER    0x01
#define REG_INTERFACE 0x02
#define REG_MODE0    0x03
#define REG_MODE1    0x04
#define REG_MODE2    0x05
#define REG_INPMUX   0x06   // input mux
#define REG_OFCAL0   0x07
#define REG_OFCAL1   0x08
#define REG_OFCAL2   0x09
#define REG_FSCAL0   0x0A
#define REG_FSCAL1   0x0B
#define REG_FSCAL2   0x0C
#define REG_IDACMUX  0x0D
#define REG_IDACMAG  0x0E
#define REG_REFMUX   0x0F
#define REG_TDACP    0x10
#define REG_TDACN    0x11
#define REG_GPIOCON  0x12
#define REG_GPIODIR  0x13
#define REG_GPIODAT  0x14

// MODE2 — data rate + PGA
// Bits[7:4] = GAIN, Bits[3:0] = DR
#define DR_2P5      0x00
#define DR_5        0x01
#define DR_10       0x02
#define DR_16P6     0x03
#define DR_20       0x04
#define DR_50       0x05
#define DR_60       0x06
#define DR_100      0x07
#define DR_400      0x08
#define DR_1200     0x09
#define DR_2400     0x0A
#define DR_4800     0x0B
#define DR_7200     0x0C
#define DR_14400    0x0D
#define DR_19200    0x0E
#define DR_38400    0x0F

// GAIN values (MODE2 bits[7:4])
#define GAIN_1      0x00
#define GAIN_2      0x10
#define GAIN_4      0x20
#define GAIN_8      0x30
#define GAIN_16     0x40
#define GAIN_32     0x50

// MODE1 — digital filter (bits[7:5])
// FIR is only valid for DR ≤ 60 SPS (DR_2P5 … DR_60)
#define FILTER_SINC1  0x00   // 000_00000
#define FILTER_SINC2  0x20   // 001_00000
#define FILTER_SINC3  0x40   // 010_00000
#define FILTER_SINC4  0x60   // 011_00000
#define FILTER_FIR    0x80   // 100_00000

// ── Active filter — uncomment exactly one ────────────────
//#define ADC_FILTER  FILTER_SINC1
//#define ADC_FILTER  FILTER_SINC2
//#define ADC_FILTER  FILTER_SINC3
//#define ADC_FILTER  FILTER_SINC4
#define ADC_FILTER  FILTER_SINC4

// INPMUX — channel select
// Bits[7:4] = positive input (MUXP), Bits[3:0] = negative input (MUXN)
// Compose as: (MUXP_* | MUXN_*)
#define MUXP_AIN0    0x00
#define MUXP_AIN1    0x10
#define MUXP_AIN2    0x20
#define MUXP_AIN3    0x30
#define MUXP_AIN4    0x40
#define MUXP_AIN5    0x50
#define MUXP_AIN6    0x60
#define MUXP_AIN7    0x70
#define MUXP_AIN8    0x80
#define MUXP_AIN9    0x90
#define MUXP_AINCOM  0xA0

#define MUXN_AIN0    0x00
#define MUXN_AIN1    0x01
#define MUXN_AIN2    0x02
#define MUXN_AIN3    0x03
#define MUXN_AIN4    0x04
#define MUXN_AIN5    0x05
#define MUXN_AIN6    0x06
#define MUXN_AIN7    0x07
#define MUXN_AIN8    0x08
#define MUXN_AIN9    0x09
#define MUXN_AINCOM  0x0A

// Convenience presets
#define MUX_CH0_AINCOM  (MUXP_AIN0 | MUXN_AINCOM)
#define MUX_CH1_AINCOM  (MUXP_AIN1 | MUXN_AINCOM)
#define MUX_CH0_CH1     (MUXP_AIN0 | MUXN_AIN1)
#define MUX_CH2_CH3     (MUXP_AIN2 | MUXN_AIN3)

// ── Derived DRDY parameters (auto-computed from ADC_DR) ──
// ADC_DR_SPS          : nominal output rate in samples/s
// ADC_DRDY_TIMEOUT_MS : safe DRDY wait timeout (≥ 2× period, min 5 ms)
#if   ADC_DR == DR_2P5
#  define ADC_DR_SPS             3
#  define ADC_DRDY_TIMEOUT_MS    1000
#elif ADC_DR == DR_5
#  define ADC_DR_SPS             5
#  define ADC_DRDY_TIMEOUT_MS    500
#elif ADC_DR == DR_10
#  define ADC_DR_SPS             10
#  define ADC_DRDY_TIMEOUT_MS    250
#elif ADC_DR == DR_16P6
#  define ADC_DR_SPS             17
#  define ADC_DRDY_TIMEOUT_MS    150
#elif ADC_DR == DR_20
#  define ADC_DR_SPS             20
#  define ADC_DRDY_TIMEOUT_MS    125
#elif ADC_DR == DR_50
#  define ADC_DR_SPS             50
#  define ADC_DRDY_TIMEOUT_MS    50
#elif ADC_DR == DR_60
#  define ADC_DR_SPS             60
#  define ADC_DRDY_TIMEOUT_MS    40
#elif ADC_DR == DR_100
#  define ADC_DR_SPS             100
#  define ADC_DRDY_TIMEOUT_MS    25
#elif ADC_DR == DR_400
#  define ADC_DR_SPS             400
#  define ADC_DRDY_TIMEOUT_MS    10
#elif ADC_DR == DR_1200
#  define ADC_DR_SPS             1200
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_2400
#  define ADC_DR_SPS             2400
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_4800
#  define ADC_DR_SPS             4800
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_7200
#  define ADC_DR_SPS             7200
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_14400
#  define ADC_DR_SPS             14400
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_19200
#  define ADC_DR_SPS             19200
#  define ADC_DRDY_TIMEOUT_MS    5
#elif ADC_DR == DR_38400
#  define ADC_DR_SPS             38400
#  define ADC_DRDY_TIMEOUT_MS    5
#else
#  error "ADC_DR not set — uncomment exactly one DR_* in ads1263.h"
#endif

// ── REFMUX — reference source select ────────────────────
// Bits[5:3]=RMUXP  Bits[2:0]=RMUXN
#define REFMUX_INTERNAL       0x00   // internal 2.5V ref (default)
#define REFMUX_AIN0_AIN1      0x09   // AIN0=REFP, AIN1=REFN  (ratiometric via signal pins)
#define REFMUX_AIN2_AIN3      0x12   // AIN2=REFP, AIN3=REFN  (ratiometric via dedicated pins)
#define REFMUX_AVDD_AVSS      0x24   // AVDD/AVSS as reference

// ── Reference voltage and conversion ─────────────────────
// Set VREF to the actual excitation voltage when using ratiometric
#define VREF 3.3f  // internal reference voltage (or bridge excitation voltage for ratiometric)

// ADC_GAIN bits[7:4] encode log2(gain): 0x00→1, 0x10→2, 0x20→4 … 0x50→32
#define ADC_GAIN_FACTOR  (1 << (ADC_GAIN >> 4))

// ── Runtime DR lookup (index = DR_* value 0x00–0x0F) ─────
static const uint32_t ADS1263_DR_SPS[16] = {
    3, 5, 10, 17, 20, 50, 60, 100, 400, 1200, 2400, 4800, 7200, 14400, 19200, 38400
};
static const int ADS1263_DR_TIMEOUT_MS[16] = {
    1000, 500, 250, 150, 125, 50, 40, 25, 10, 5, 5, 5, 5, 5, 5, 5
};

// ── Runtime gain lookup (index = gain_reg >> 4, i.e. 0–5) ─
static const int ADS1263_GAIN_FACTOR[6] = { 1, 2, 4, 8, 16, 32 };
// gain register byte for each index
static const uint8_t ADS1263_GAIN_REG[6] = {
    GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32
};
static const char * const ADS1263_GAIN_NAME[6] = {
    "×1", "×2", "×4", "×8", "×16", "×32"
};

// ── Runtime filter lookup (index 0–4 = SINC1..FIR) ───────
// MODE1 upper byte value for each filter index
static const uint8_t ADS1263_FILTER_MODE1[5] = {
    FILTER_SINC1, FILTER_SINC2, FILTER_SINC3, FILTER_SINC4, FILTER_FIR
};
static const char * const ADS1263_FILTER_NAME[5] = {
    "SINC1", "SINC2", "SINC3", "SINC4", "FIR"
};

// ── Function prototypes ───────────────────────────────────
float raw_to_volts(int32_t raw);
int64_t ads1263_init(struct gpiod_chip * pChip);
int64_t ads1263_deinit(void);
int32_t ads1263_read(void);
void    ads1263_set_dr(uint8_t dr);
void    ads1263_set_filter(uint8_t filter_idx);
void    ads1263_set_gain(uint8_t gain_reg);
void    ads1263_set_refmux(uint8_t refmux);
void    ads1263_set_inpmux(uint8_t inpmux);
uint8_t ads1263_read_reg(uint8_t reg);
void    ads1263_write_reg_raw(uint8_t reg, uint8_t val);
void    ads1263_conv_start(void);
void    ads1263_conv_stop(void);

#endif // ADS1263_H