#ifndef RP1_DMA_H
#define RP1_DMA_H

#include <stddef.h>
#include <stdint.h>

// ── RP1 DREQ numbers (from RP1 peripherals datasheet / rp1.dtsi) ─────────────
#define RP1_DREQ_SPI0_RX   3
#define RP1_DREQ_SPI0_TX   4
#define RP1_DREQ_SPI1_RX   5
#define RP1_DREQ_SPI1_TX   6

typedef struct rp1_dma rp1_dma_t;

/*
 * Open RP1 DMA for SPI transfers.
 *   dreq_rx / dreq_tx : DREQ numbers for the SPI peripheral (see above).
 *
 * Requires:
 *   - /dev/mem access  (run as root, or boot with iomem=relaxed)
 *   - /dev/dma_heap/linux,cma  (kernel CONFIG_DMABUF_HEAPS_CMA=y)
 *
 * The function finds the RP1 PCIe BAR0 automatically via sysfs,
 * maps the DMA and SPI0 register blocks, and allocates two CMA
 * buffers (TX and RX) for DMA transfers.
 */
rp1_dma_t *rp1_dma_open(int dreq_rx, int dreq_tx);
void       rp1_dma_close(rp1_dma_t *d);

/*
 * Full-duplex SPI transfer via RP1 DMA (no CPU involvement during transfer).
 *   tx_buf : data to send  (NULL → sends zeros)
 *   rx_buf : receive buffer (NULL → discards)
 *   len    : byte count  (max 4095 per PL080 transfer-size field)
 *
 * Returns 0 on success, -1 on timeout / error.
 *
 * NOTE: ads1263_init() must be called first to configure the ADS1263
 * registers via spidev.  After init, ads1263.c closes the spidev fd
 * so the SPI hardware is free for DMA access.
 */
int rp1_dma_spi_transfer(rp1_dma_t *d,
                          const void *tx_buf, void *rx_buf, size_t len);

#endif // RP1_DMA_H
