#include <libcaer/libcaer.h>
#include <libcaer/devices/dvxplorer.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>

extern "C" {
#include "flow_unpack.h"
}

// ─── AXI DMA register offsets (Xilinx PG021) ─────────────────────────────────
#define DMA_BASE_ADDR       0xA0000000UL   // adjust if your block design differs
#define DMA_MAP_SIZE        0x10000UL

#define MM2S_DMACR          0x00
#define MM2S_DMASR          0x04
#define MM2S_SA             0x18
#define MM2S_SA_MSB         0x1C           // upper 32 bits (ZynqMP 64-bit)
#define MM2S_LENGTH         0x28

#define S2MM_DMACR          0x30
#define S2MM_DMASR          0x34
#define S2MM_DA             0x48
#define S2MM_DA_MSB         0x4C           // upper 32 bits (ZynqMP 64-bit)
#define S2MM_LENGTH         0x58

#define DMACR_RS            (1u << 0)      // Run/Stop bit
#define DMASR_IOC_IRQ       (1u << 12)     // Interrupt on Complete

// ─── udmabuf sizes ───────────────────────────────────────────────────────────
#define UDMABUF_SIZE        65536UL        // 64 KB, matches your device tree
#define UDMABUF_TX          "/dev/udmabuf0"
#define UDMABUF_RX          "/dev/udmabuf1"
#define UDMABUF_TX_PHYS     "/sys/class/udmabuf/udmabuf0/phys_addr"
#define UDMABUF_RX_PHYS     "/sys/class/udmabuf/udmabuf1/phys_addr"

// ─── Visualizer tuning ───────────────────────────────────────────────────────
#define DECAY_STEP          20             // per-frame brightness fade (0-255)
#define FLOW_SCALE          10.0f          // arrow length multiplier

static atomic_bool g_shutdown = ATOMIC_VAR_INIT(false);

static void on_signal(int) {
    atomic_store(&g_shutdown, true);
}

static void on_usb_shutdown(void *) {
    atomic_store(&g_shutdown, true);
}

// ─── Read physical address from sysfs ────────────────────────────────────────
static uint64_t read_phys_addr(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return 0; }
    uint64_t addr = 0;
    fscanf(f, "0x%lx", &addr);
    fclose(f);
    return addr;
}

// ─── DMA register helpers ────────────────────────────────────────────────────
static inline void reg_write(volatile uint32_t *base, uint32_t offset, uint32_t val) {
    base[offset / 4] = val;
}
static inline uint32_t reg_read(volatile uint32_t *base, uint32_t offset) {
    return base[offset / 4];
}

static void dma_wait_complete(volatile uint32_t *base, uint32_t sr_offset) {
    while (!(reg_read(base, sr_offset) & DMASR_IOC_IRQ)) {}
    reg_write(base, sr_offset, reg_read(base, sr_offset) | DMASR_IOC_IRQ); // clear
}

int main(void) {
    // ── Signal handlers ──────────────────────────────────────────────────────
    struct sigaction sa;
    sa.sa_handler = on_signal;
    sa.sa_flags   = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);

    // ── Open DVXplorer ───────────────────────────────────────────────────────
    caerDeviceHandle cam = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, NULL);
    if (!cam) { fprintf(stderr, "Failed to open DVXplorer\n"); return EXIT_FAILURE; }

    struct caer_dvx_info info = caerDVXplorerInfoGet(cam);
    printf("%s --- DVS %dx%d\n", info.deviceString, info.dvsSizeX, info.dvsSizeY);

    caerDeviceSendDefaultConfig(cam);
    caerDeviceDataStart(cam, NULL, NULL, NULL, on_usb_shutdown, NULL);
    caerDeviceConfigSet(cam, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

    // ── Map udmabuf TX (events → FPGA) ──────────────────────────────────────
    int fd_tx = open(UDMABUF_TX, O_RDWR);
    if (fd_tx < 0) { perror(UDMABUF_TX); goto cleanup_cam; }
    void *tx_virt = mmap(NULL, UDMABUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_tx, 0);
    if (tx_virt == MAP_FAILED) { perror("mmap tx"); goto cleanup_tx_fd; }
    uint64_t tx_phys = read_phys_addr(UDMABUF_TX_PHYS);
    printf("udmabuf TX: virt=%p  phys=0x%lx\n", tx_virt, tx_phys);

    // ── Map udmabuf RX (flow ← FPGA) ────────────────────────────────────────
    int fd_rx = open(UDMABUF_RX, O_RDWR);
    if (fd_rx < 0) { perror(UDMABUF_RX); goto cleanup_tx_map; }
    void *rx_virt = mmap(NULL, UDMABUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_rx, 0);
    if (rx_virt == MAP_FAILED) { perror("mmap rx"); goto cleanup_rx_fd; }
    uint64_t rx_phys = read_phys_addr(UDMABUF_RX_PHYS);
    printf("udmabuf RX: virt=%p  phys=0x%lx\n", rx_virt, rx_phys);

    // ── Map AXI DMA registers via /dev/mem ───────────────────────────────────
    int fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_mem < 0) { perror("/dev/mem"); goto cleanup_rx_map; }
    volatile uint32_t *dma = (volatile uint32_t *)mmap(
        NULL, DMA_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, DMA_BASE_ADDR);
    if (dma == MAP_FAILED) { perror("mmap dma"); goto cleanup_mem_fd; }

    // Start both DMA channels (set Run/Stop bit)
    reg_write(dma, MM2S_DMACR, DMACR_RS);
    reg_write(dma, S2MM_DMACR, DMACR_RS);

    // ── OpenCV window ────────────────────────────────────────────────────────
    int W = info.dvsSizeX, H = info.dvsSizeY;
    cv::Mat frame = cv::Mat::zeros(H, W, CV_8UC3);
    cv::namedWindow("DVXplorer | Events + Optical Flow", cv::WINDOW_AUTOSIZE);

    // ── Main loop ────────────────────────────────────────────────────────────
    while (!atomic_load_explicit(&g_shutdown, memory_order_relaxed)) {

        // --- Collect events from camera and write them to TX udmabuf ----------
        caerEventPacketContainer pkt = caerDeviceDataGet(cam);
        if (!pkt) continue;

        uint64_t *tx_words = (uint64_t *)tx_virt;
        int n_words = 0;
        int max_words = (int)(UDMABUF_SIZE / sizeof(uint64_t));

        int32_t n_packets = caerEventPacketContainerGetEventPacketsNumber(pkt);
        for (int32_t pi = 0; pi < n_packets; pi++) {
            caerEventPacketHeader hdr = caerEventPacketContainerGetEventPacket(pkt, pi);
            if (!hdr || caerEventPacketHeaderGetEventType(hdr) != POLARITY_EVENT) continue;

            caerPolarityEventPacket polpkt = (caerPolarityEventPacket)hdr;
            int32_t n_ev = caerEventPacketHeaderGetEventNumber(hdr);

            for (int32_t ei = 0; ei < n_ev && n_words < max_words; ei++) {
                caerPolarityEvent ev = caerPolarityEventPacketGetEvent(polpkt, ei);
                if (!caerPolarityEventIsValid(ev)) continue;

                int32_t  ts  = caerPolarityEventGetTimestamp(ev);
                uint16_t x   = caerPolarityEventGetX(ev);
                uint16_t y   = caerPolarityEventGetY(ev);
                bool     pol = caerPolarityEventGetPolarity(ev);

                // Paint event on canvas
                cv::Vec3b &px = frame.at<cv::Vec3b>((int)y, (int)x);
                px = pol ? cv::Vec3b(0, 255, 0)   // ON  → green
                         : cv::Vec3b(0, 0, 255);  // OFF → red

                // Pack into TX buffer
                uint64_t word = 0;
                word |= ((uint64_t)(uint32_t)ts)         << 32;
                word |= ((uint64_t)(x & 0xFFFu))         << 20;
                word |= ((uint64_t)(y & 0xFFFu))         <<  8;
                word |= ((uint64_t)(pol ? 1u : 0u))      <<  7;
                tx_words[n_words++] = word;
            }
        }
        caerEventPacketContainerFree(pkt);

        if (n_words == 0) goto render;

        // --- DMA TX: send events to FPGA (MM2S) ------------------------------
        uint32_t tx_len = (uint32_t)(n_words * sizeof(uint64_t));
        reg_write(dma, MM2S_SA,     (uint32_t)(tx_phys & 0xFFFFFFFFUL));
        reg_write(dma, MM2S_SA_MSB, (uint32_t)(tx_phys >> 32));
        reg_write(dma, MM2S_LENGTH, tx_len);           // triggers transfer

        // --- DMA RX: receive flow vectors from FPGA (S2MM) -------------------
        // Pre-arm the RX channel for the same number of words
        uint32_t rx_len = tx_len;
        reg_write(dma, S2MM_DA,     (uint32_t)(rx_phys & 0xFFFFFFFFUL));
        reg_write(dma, S2MM_DA_MSB, (uint32_t)(rx_phys >> 32));
        reg_write(dma, S2MM_LENGTH, rx_len);           // triggers receive

        // Wait for both transfers to complete
        dma_wait_complete(dma, MM2S_DMASR);
        dma_wait_complete(dma, S2MM_DMASR);

        // --- Unpack flow vectors and draw arrows ------------------------------
        uint64_t *rx_words = (uint64_t *)rx_virt;
        int n_flow = (int)(rx_len / sizeof(uint64_t));

        for (int fi = 0; fi < n_flow; fi++) {
            flow_vector_t fv;
            flow_unpack(rx_words[fi], &fv);
            if (fv.x >= (uint16_t)W || fv.y >= (uint16_t)H) continue;

            float dx = (float)fv.vx / 256.0f * FLOW_SCALE;
            float dy = (float)fv.vy / 256.0f * FLOW_SCALE;
            cv::Point2f origin((float)fv.x, (float)fv.y);
            cv::Point2f tip(origin.x + dx, origin.y + dy);
            cv::arrowedLine(frame, origin, tip,
                            cv::Scalar(0, 255, 255), 1, cv::LINE_AA, 0, 0.3f);
        }

render:
        // --- Fade old events and display -------------------------------------
        for (int r = 0; r < frame.rows; r++) {
            cv::Vec3b *row = frame.ptr<cv::Vec3b>(r);
            for (int c = 0; c < frame.cols; c++) {
                cv::Vec3b &px = row[c];
                for (int ch = 0; ch < 3; ch++)
                    px[ch] = px[ch] > DECAY_STEP ? px[ch] - DECAY_STEP : 0;
            }
        }

        cv::imshow("DVXplorer | Events + Optical Flow", frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
            atomic_store(&g_shutdown, true);
    }

    // ── Cleanup ──────────────────────────────────────────────────────────────
    munmap((void *)dma, DMA_MAP_SIZE);
cleanup_mem_fd:
    close(fd_mem);
cleanup_rx_map:
    munmap(rx_virt, UDMABUF_SIZE);
cleanup_rx_fd:
    close(fd_rx);
cleanup_tx_map:
    munmap(tx_virt, UDMABUF_SIZE);
cleanup_tx_fd:
    close(fd_tx);
cleanup_cam:
    caerDeviceDataStop(cam);
    caerDeviceClose(&cam);
    cv::destroyAllWindows();

    printf("Shutdown complete.\n");
    return EXIT_SUCCESS;
}
