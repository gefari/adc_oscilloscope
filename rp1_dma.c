#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-heap.h>

#include "rp1_dma.h"

// ── RP1 PCI identity ──────────────────────────────────────────────────────────
#define RP1_PCI_VENDOR   0x1de4u
#define RP1_PCI_DEVICE   0x0001u

// ── RP1 peripheral offsets (RP1 peripherals datasheet §1) ────────────────────
#define RP1_SPI0_OFFSET  0x0050c000UL
#define RP1_DMA_OFFSET   0x00188000UL
#define RP1_MMAP_SIZE    0x00400000UL   // map 4 MB of RP1 space

// ── PL022 SSP register offsets ───────────────────────────────────────────────
#define SSP_CR0          0x000
#define SSP_CR1          0x004
#define SSP_DR           0x008   // TX / RX FIFO
#define SSP_SR           0x00C
#define SSP_CPSR         0x010
#define SSP_DMACR        0x024
#define SSP_DMACR_RXDMAE (1U << 0)
#define SSP_DMACR_TXDMAE (1U << 1)
#define SSP_CR1_SSE      (1U << 1)  // SSP enable bit
#define SSP_CR0_DSS_8BIT 0x7        // 8-bit data
#define SSP_CR0_SPH      (1U << 7)  // SPI mode 1: CPHA=1

// ── RP1 DMA (PL080) global registers ─────────────────────────────────────────
#define DMA_INT_STATUS   0x000
#define DMA_INT_TC_STAT  0x004
#define DMA_INT_TC_CLR   0x008
#define DMA_INT_ERR_STAT 0x00C
#define DMA_INT_ERR_CLR  0x010
#define DMA_CONFIG       0x030
#define DMAC_EN          (1U << 0)
#define DMAC_AHB1_EN     (1U << 1)  // enable AHB master 1 (internal)
#define DMAC_AHB2_EN     (1U << 2)  // enable AHB master 2 (external PCIe RAM)

// ── PL080 per-channel registers (channel n offset within DMA block) ───────────
#define DMA_CH_BASE(n)  (0x100u + (unsigned)(n) * 0x20u)
#define DMA_CH_SRC(n)   (DMA_CH_BASE(n) + 0x00u)
#define DMA_CH_DST(n)   (DMA_CH_BASE(n) + 0x04u)
#define DMA_CH_LLI(n)   (DMA_CH_BASE(n) + 0x08u)
#define DMA_CH_CTRL(n)  (DMA_CH_BASE(n) + 0x0Cu)
#define DMA_CH_CFG(n)   (DMA_CH_BASE(n) + 0x10u)

// PL080 CTRL register bits
#define CTRL_SIZE(n)    ((uint32_t)(n) & 0xFFFu)
#define CTRL_SBS(b)     ((uint32_t)(b) << 12)   // source burst size
#define CTRL_DBS(b)     ((uint32_t)(b) << 15)   // dest burst size
#define CTRL_SWIDTH(w)  ((uint32_t)(w) << 18)   // 0=byte 1=hword 2=word
#define CTRL_DWIDTH(w)  ((uint32_t)(w) << 21)
#define CTRL_SRC_AHB2   (1U << 24)  // source from AHB2 (system RAM via PCIe)
#define CTRL_DST_AHB2   (1U << 25)  // dest to AHB2
#define CTRL_SI         (1U << 26)  // source address increment
#define CTRL_DI         (1U << 27)  // dest address increment
#define CTRL_TC_INT     (1U << 31)  // terminal count interrupt enable

// PL080 CFG register bits
#define CFG_EN          (1U << 0)
#define CFG_SRC_PER(p)  ((uint32_t)(p) << 1)   // bits [5:1]
#define CFG_DST_PER(p)  ((uint32_t)(p) << 6)   // bits [10:6]
#define CFG_FLOW(f)     ((uint32_t)(f) << 11)  // bits [13:11]
#define CFG_ITC         (1U << 15)  // unmask TC interrupt
#define FLOW_M2P        1u          // memory → peripheral
#define FLOW_P2M        2u          // peripheral → memory

// DMA channel assignment
#define CH_TX  0
#define CH_RX  1

// DMA buffer size
#define DMA_BUF_BYTES  64u

struct rp1_dma {
    int              devmem_fd;
    volatile uint8_t *rp1_base;   // mmap base of RP1 region
    volatile uint32_t *dma;       // points into rp1_base at DMA_OFFSET
    volatile uint32_t *spi;       // points into rp1_base at SPI0_OFFSET

    int    tx_heap_fd;
    int    rx_heap_fd;
    void  *tx_vaddr;
    void  *rx_vaddr;
    uint64_t tx_paddr;
    uint64_t rx_paddr;

    int    dreq_rx;
    int    dreq_tx;
};

// ── Helpers ───────────────────────────────────────────────────────────────────

static inline uint32_t reg_rd(volatile uint32_t *base, unsigned off) {
    return base[off / 4];
}
static inline void reg_wr(volatile uint32_t *base, unsigned off, uint32_t v) {
    base[off / 4] = v;
}

// Find RP1 PCIe BAR0 physical address via sysfs
static uint64_t rp1_bar0(void) {
    DIR *dir = opendir("/sys/bus/pci/devices");
    if (!dir) return 0;

    struct dirent *e;
    while ((e = readdir(dir)) != NULL) {
        if (e->d_name[0] == '.') continue;

        char path[300];
        unsigned v = 0, d = 0;
        FILE *f;

        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/vendor", e->d_name);
        f = fopen(path, "r"); if (!f) continue;
        fscanf(f, "%x", &v); fclose(f);
        if (v != RP1_PCI_VENDOR) continue;

        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/device", e->d_name);
        f = fopen(path, "r"); if (!f) continue;
        fscanf(f, "%x", &d); fclose(f);
        if (d != RP1_PCI_DEVICE) continue;

        // First line of 'resource' is BAR0: "start end flags"
        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource", e->d_name);
        f = fopen(path, "r"); if (!f) { closedir(dir); return 0; }
        uint64_t start = 0, end = 0, flags = 0;
        fscanf(f, "0x%lx 0x%lx 0x%lx", &start, &end, &flags);
        fclose(f);
        closedir(dir);
        return start;
    }
    closedir(dir);
    return 0;
}

// Get physical address for a virtual address via /proc/self/pagemap
static uint64_t virt_to_phys(void *vaddr) {
    long page_size = sysconf(_SC_PAGE_SIZE);
    uint64_t vpage = (uint64_t)(uintptr_t)vaddr / (uint64_t)page_size;
    uint64_t entry = 0;

    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) return 0;
    pread(fd, &entry, sizeof(entry), (off_t)(vpage * sizeof(entry)));
    close(fd);

    if (!(entry & (1ULL << 63))) return 0;  // page not present
    uint64_t pfn = entry & ((1ULL << 55) - 1);
    return pfn * (uint64_t)page_size + ((uint64_t)(uintptr_t)vaddr & (uint64_t)(page_size - 1));
}

// Allocate a DMA-coherent buffer via /dev/dma_heap/linux,cma
static int alloc_dma_buf(size_t size, void **vaddr, int *out_fd) {
    static const char *heaps[] = {
        "/dev/dma_heap/linux,cma",
        "/dev/dma_heap/reserved",
        "/dev/dma_heap/system",
        NULL
    };
    int heap_fd = -1;
    for (int i = 0; heaps[i]; i++) {
        heap_fd = open(heaps[i], O_RDWR);
        if (heap_fd >= 0) break;
    }
    if (heap_fd < 0) { perror("dma_heap open"); return -1; }

    struct dma_heap_allocation_data data = {
        .len      = size,
        .fd_flags = O_RDWR | O_CLOEXEC,
        .heap_flags = 0,
    };
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data) < 0) {
        perror("DMA_HEAP_IOCTL_ALLOC"); close(heap_fd); return -1;
    }
    close(heap_fd);

    *out_fd = data.fd;
    *vaddr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, *out_fd, 0);
    if (*vaddr == MAP_FAILED) { perror("mmap dma buf"); close(*out_fd); return -1; }
    return 0;
}

// ── Public API ────────────────────────────────────────────────────────────────

rp1_dma_t *rp1_dma_open(int dreq_rx, int dreq_tx) {
    rp1_dma_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->dreq_rx = dreq_rx;
    d->dreq_tx = dreq_tx;

    // Locate RP1 BAR0
    uint64_t bar0 = rp1_bar0();
    if (!bar0) { fprintf(stderr, "rp1_dma: RP1 not found in PCI sysfs\n"); goto err; }
    printf("rp1_dma: RP1 BAR0 = 0x%lx\n", bar0);

    // Map RP1 register space via /dev/mem
    d->devmem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (d->devmem_fd < 0) { perror("/dev/mem"); goto err; }

    d->rp1_base = mmap(NULL, RP1_MMAP_SIZE, PROT_READ | PROT_WRITE,
                       MAP_SHARED, d->devmem_fd, (off_t)bar0);
    if (d->rp1_base == MAP_FAILED) { perror("mmap RP1"); goto err; }

    d->dma = (volatile uint32_t *)(d->rp1_base + RP1_DMA_OFFSET);
    d->spi = (volatile uint32_t *)(d->rp1_base + RP1_SPI0_OFFSET);

    // Enable RP1 DMA controller
    reg_wr(d->dma, DMA_CONFIG, DMAC_EN | DMAC_AHB1_EN | DMAC_AHB2_EN);

    // Allocate TX and RX DMA buffers
    if (alloc_dma_buf(DMA_BUF_BYTES, &d->tx_vaddr, &d->tx_heap_fd) < 0) goto err;
    if (alloc_dma_buf(DMA_BUF_BYTES, &d->rx_vaddr, &d->rx_heap_fd) < 0) goto err;

    d->tx_paddr = virt_to_phys(d->tx_vaddr);
    d->rx_paddr = virt_to_phys(d->rx_vaddr);
    if (!d->tx_paddr || !d->rx_paddr) {
        fprintf(stderr, "rp1_dma: failed to get physical addresses\n"); goto err;
    }
    printf("rp1_dma: TX buf paddr=0x%lx  RX buf paddr=0x%lx\n",
           d->tx_paddr, d->rx_paddr);

    // Enable SPI DMA request lines
    uint32_t cr1 = reg_rd(d->spi, SSP_CR1);
    reg_wr(d->spi, SSP_CR1, cr1 & ~SSP_CR1_SSE);       // disable SSP briefly
    reg_wr(d->spi, SSP_DMACR, SSP_DMACR_RXDMAE | SSP_DMACR_TXDMAE);
    reg_wr(d->spi, SSP_CR1, cr1 | SSP_CR1_SSE);        // re-enable

    return d;
err:
    rp1_dma_close(d);
    return NULL;
}

void rp1_dma_close(rp1_dma_t *d) {
    if (!d) return;
    if (d->rp1_base && d->rp1_base != MAP_FAILED)
        munmap((void *)d->rp1_base, RP1_MMAP_SIZE);
    if (d->tx_vaddr && d->tx_vaddr != MAP_FAILED)
        munmap(d->tx_vaddr, DMA_BUF_BYTES);
    if (d->rx_vaddr && d->rx_vaddr != MAP_FAILED)
        munmap(d->rx_vaddr, DMA_BUF_BYTES);
    if (d->tx_heap_fd > 0) close(d->tx_heap_fd);
    if (d->rx_heap_fd > 0) close(d->rx_heap_fd);
    if (d->devmem_fd > 0)  close(d->devmem_fd);
    free(d);
}

int rp1_dma_spi_transfer(rp1_dma_t *d,
                          const void *tx_buf, void *rx_buf, size_t len) {
    if (len > DMA_BUF_BYTES || len > 4095) return -1;

    // Fill TX buffer
    if (tx_buf) memcpy(d->tx_vaddr, tx_buf, len);
    else        memset(d->tx_vaddr, 0, len);

    // Physical address of SPI DR register (RP1-internal AHB1 address)
    uint32_t spi_dr_iaddr = (uint32_t)(RP1_SPI0_OFFSET + SSP_DR);

    // Clear pending TC/error flags for both channels
    reg_wr(d->dma, DMA_INT_TC_CLR,   (1U << CH_TX) | (1U << CH_RX));
    reg_wr(d->dma, DMA_INT_ERR_CLR,  (1U << CH_TX) | (1U << CH_RX));

    // ── RX channel: SPI FIFO (AHB1) → RAM (AHB2) ────────────────────────────
    reg_wr(d->dma, DMA_CH_CFG(CH_RX), 0);  // disable while configuring
    reg_wr(d->dma, DMA_CH_SRC(CH_RX), spi_dr_iaddr);
    reg_wr(d->dma, DMA_CH_DST(CH_RX), (uint32_t)d->rx_paddr);
    reg_wr(d->dma, DMA_CH_LLI(CH_RX), 0);
    reg_wr(d->dma, DMA_CH_CTRL(CH_RX),
           CTRL_SIZE(len) | CTRL_SWIDTH(0) | CTRL_DWIDTH(0) |
           CTRL_DST_AHB2 | CTRL_DI | CTRL_TC_INT);
    reg_wr(d->dma, DMA_CH_CFG(CH_RX),
           CFG_SRC_PER(d->dreq_rx) | CFG_FLOW(FLOW_P2M) | CFG_ITC | CFG_EN);

    // ── TX channel: RAM (AHB2) → SPI FIFO (AHB1) ────────────────────────────
    reg_wr(d->dma, DMA_CH_CFG(CH_TX), 0);
    reg_wr(d->dma, DMA_CH_SRC(CH_TX), (uint32_t)d->tx_paddr);
    reg_wr(d->dma, DMA_CH_DST(CH_TX), spi_dr_iaddr);
    reg_wr(d->dma, DMA_CH_LLI(CH_TX), 0);
    reg_wr(d->dma, DMA_CH_CTRL(CH_TX),
           CTRL_SIZE(len) | CTRL_SWIDTH(0) | CTRL_DWIDTH(0) |
           CTRL_SRC_AHB2 | CTRL_SI | CTRL_TC_INT);
    reg_wr(d->dma, DMA_CH_CFG(CH_TX),
           CFG_DST_PER(d->dreq_tx) | CFG_FLOW(FLOW_M2P) | CFG_ITC | CFG_EN);

    // Poll RX TC flag (peripheral-to-memory, RX determines transfer end)
    int timeout = 50000;
    while (!(reg_rd(d->dma, DMA_INT_TC_STAT) & (1U << CH_RX)) && --timeout)
        ;

    reg_wr(d->dma, DMA_INT_TC_CLR,  (1U << CH_TX) | (1U << CH_RX));

    if (timeout <= 0) { fprintf(stderr, "rp1_dma: transfer timeout\n"); return -1; }

    if (rx_buf) memcpy(rx_buf, d->rx_vaddr, len);
    return 0;
}
