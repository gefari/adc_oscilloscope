#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/spi/spidev.h>

#include "ads1263.h"

#define SPI_DEV       "/dev/spidev0.0"

#define PIN_RESET     18         // GPIO BCM27 — connect to RESET

#define SPI_HZ        1000000    // 1MHz — safe for ADS1263


static struct gpiod_line_request *rst_req  = NULL;
static int  spi_fd   = -1;

static uint8_t rt_gain = ADC_GAIN;   // current GAIN_* register value
static uint8_t rt_dr   = ADC_DR;     // current DR_*  register value
static float   rt_vref = VREF;       // current reference voltage (V)

/* LOCAL FUNCTION DECLARATIONS */
static void spi_cmd(uint8_t cmd);
static void spi_transfer(uint8_t *tx, uint8_t *rx, int len);
static void write_reg(uint8_t reg, uint8_t val);
static uint8_t read_reg(uint8_t reg);

/* PUBLIC WRAPPERS FOR RAW REGISTER ACCESS */
uint8_t ads1263_read_reg(uint8_t reg)  { return read_reg(reg); }
void    ads1263_write_reg_raw(uint8_t reg, uint8_t val) { write_reg(reg, val); }
void    ads1263_conv_start(void) { spi_cmd(ADS1263_CMD_START1); }
void    ads1263_conv_stop(void)  { spi_cmd(ADS1263_CMD_STOP1); }

/* PUBLIC FUNCTION IMPLEMENTATION */

float raw_to_volts(int32_t raw) {
    int gain_factor = 1 << (rt_gain >> 4);
    return (float)raw * rt_vref / ((float)gain_factor * (float)(1UL << 31));
}

// ── Read one 32-bit conversion result ────────────────────
// Returns raw signed 32-bit value
int32_t ads1263_read(void) {
    // RDATA1 command + read 5 bytes: status + 4 data bytes
    uint8_t tx[6] = { ADS1263_CMD_RDATA1, 0, 0, 0, 0, 0 };
    uint8_t rx[6] = { 0 };
    spi_transfer(tx, rx, 6);

    // rx[0] = dummy (cmd echo)
    // rx[1] = status byte
    // rx[2..5] = 32-bit result MSB first
    int32_t raw = (int32_t)(
        ((uint32_t)rx[2] << 24) |
        ((uint32_t)rx[3] << 16) |
        ((uint32_t)rx[4] <<  8) |
        ((uint32_t)rx[5])
    );
    return raw;
}

void ads1263_set_dr(uint8_t dr) {
    rt_dr = dr & 0x0F;
    spi_cmd(ADS1263_CMD_STOP1);
    write_reg(REG_MODE2, rt_gain | rt_dr);
    spi_cmd(ADS1263_CMD_START1);
}

void ads1263_set_gain(uint8_t gain_reg) {
    rt_gain = gain_reg & 0xF0;
    spi_cmd(ADS1263_CMD_STOP1);
    write_reg(REG_MODE2, rt_gain | rt_dr);
    spi_cmd(ADS1263_CMD_START1);
}

void ads1263_set_refmux(uint8_t refmux) {
    spi_cmd(ADS1263_CMD_STOP1);
    write_reg(REG_REFMUX, refmux);
    spi_cmd(ADS1263_CMD_START1);
    switch (refmux) {
        case REFMUX_INTERNAL:  rt_vref = 2.5f; break;
        case 0x04:             rt_vref = 2.5f; break;  // internal+ / AVSS
        case REFMUX_AIN0_AIN1: rt_vref = 3.3f; break;
        case REFMUX_AIN2_AIN3: rt_vref = 3.3f; break;
        case REFMUX_AVDD_AVSS: rt_vref = 3.3f; break;
        default:               rt_vref = VREF;  break;
    }
}

void ads1263_set_inpmux(uint8_t inpmux) {
    spi_cmd(ADS1263_CMD_STOP1);
    write_reg(REG_INPMUX, inpmux);
    spi_cmd(ADS1263_CMD_START1);
}

void ads1263_set_filter(uint8_t filter_idx) {
    if (filter_idx > 4) return;
    spi_cmd(ADS1263_CMD_STOP1);
    write_reg(REG_MODE1, ADS1263_FILTER_MODE1[filter_idx]);
    spi_cmd(ADS1263_CMD_START1);
}

int64_t ads1263_deinit() {
    spi_cmd(ADS1263_CMD_STOP1);
    
    gpiod_line_request_release(rst_req);
    
    if (spi_fd >= 0) {
        close(spi_fd);
    }

    return 0;
}

int64_t ads1263_init(struct gpiod_chip * pChip) {
    
    int res = 0;

    spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) { perror("open spi"); return 1; }

    // SPI: Mode 1 (CPOL=0, CPHA=1), 8-bit, 1MHz
    uint8_t  spi_mode  = SPI_MODE_1;
    uint8_t  spi_bits  = 8;
    uint32_t spi_speed = SPI_HZ;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE,          &spi_mode)  < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits)  < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &spi_speed) < 0) {
        perror("spi ioctl"); return 1;
    }

    {
        struct gpiod_line_settings  *s  = gpiod_line_settings_new();
        struct gpiod_line_config    *lc = gpiod_line_config_new();
        struct gpiod_request_config *rc = gpiod_request_config_new();

        unsigned int offset = PIN_RESET;
        
        gpiod_line_settings_set_direction(s, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(s, GPIOD_LINE_VALUE_ACTIVE);
        gpiod_line_config_add_line_settings(lc, &offset, 1, s);
        gpiod_request_config_set_consumer(rc, "adc_oscilloscope");
        
        rst_req = gpiod_chip_request_lines(pChip, rc, lc);
        
        gpiod_line_settings_free(s);
        gpiod_line_config_free(lc);
        gpiod_request_config_free(rc);
    }
    if (!rst_req) { perror("request RESET"); return 1; }

    // Hardware reset
    gpiod_line_request_set_value(rst_req, PIN_RESET, GPIOD_LINE_VALUE_INACTIVE);
    usleep(200);
    gpiod_line_request_set_value(rst_req, PIN_RESET, GPIOD_LINE_VALUE_ACTIVE);
    usleep(5000);   // wait for boot


    // Software reset
    spi_cmd(ADS1263_CMD_RESET);
    usleep(5000); // wait for boot

    // Verify chip ID
    uint8_t id = read_reg(REG_ID);
    printf("ADS1263 ID: 0x%02x (expect 0x23)\n", id);
    if ((id & 0xE0) != 0x20) {
        fprintf(stderr, "ADS1263 not found!\n");
        res = -1;
    }

        // POWER: internal reference ON
    write_reg(REG_POWER, 0x11);   // INTREF=1, VBIAS=0

    // INTERFACE: timeout disabled, checksum off for speed
    write_reg(REG_INTERFACE, 0x04); // STATUS byte enabled

    // MODE0: continuous conversion, no chop/delay
    write_reg(REG_MODE0, 0x00);

    // MODE1: filter selected by ADC_FILTER
    write_reg(REG_MODE1, ADS1263_FILTER_MODE1[4]);

    // MODE2: gain and data rate
    write_reg(REG_MODE2, rt_gain | rt_dr);

    // REFMUX: ratiometric — bridge excitation on AIN2/AIN3 is the reference
    write_reg(REG_REFMUX, ADC_REF_MUX);

    // Start with CH0-CH1 differential input (default after reset) — change with cmd_inpmux from Python UI
    write_reg(REG_INPMUX, MUX_CH0_CH1);

    // START conversion
    spi_cmd(ADS1263_CMD_START1);
    usleep(1000);

    printf("ADS1263 initialized!\n");

#ifdef USE_DMA    
    // Release spidev so RP1 DMA can take over the SPI hardware directly
    close(spi_fd);
    spi_fd = -1;
#endif

    return res;
}

/* LOCAL FUNCTION IMPLEMENTATION */
// ── SPI helpers ──────────────────────────────────────────

static void spi_cmd(uint8_t cmd) {
    uint8_t tx = cmd, rx = 0;
    spi_transfer(&tx, &rx, 1);
}

static void spi_transfer(uint8_t *tx, uint8_t *rx, int len) {
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = len,
        .speed_hz      = SPI_HZ,
        .bits_per_word = 8,
        .delay_usecs   = 0,
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[3] = {
        (uint8_t)(ADS1263_CMD_WREG | reg),
        0x00,   // write 1 register
        val
    };
    uint8_t rx[3] = {0};
    spi_transfer(tx, rx, 3);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t tx[3] = {
        (uint8_t)(ADS1263_CMD_RREG | reg),
        0x00,
        0x00
    };
    uint8_t rx[3] = {0};
    spi_transfer(tx, rx, 3);
    return rx[2];
}