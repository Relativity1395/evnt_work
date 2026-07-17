#ifndef EVENT_PACK_H
#define EVENT_PACK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pack a DVS polarity event into a single 64-bit word for DMA transfer to the FPGA.
 *
 * Bit layout (adjust to match your HLS core's expected input format):
 *   [63:32]  timestamp  (uint32, microseconds)
 *   [31:20]  x          (uint12)
 *   [19:8]   y          (uint12)
 *   [7]      polarity   (1 = ON, 0 = OFF)
 *   [6:0]    reserved
 */
uint64_t event_pack(int32_t timestamp, uint16_t x, uint16_t y, bool polarity);

#ifdef __cplusplus
}
#endif

#endif /* EVENT_PACK_H */
