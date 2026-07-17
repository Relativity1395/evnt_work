#ifndef FLOW_UNPACK_H
#define FLOW_UNPACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optical flow vector output from the FPGA.
 *
 * vx/vy are signed Q8.8 fixed-point (divide by 256.0 to get float pixels/s).
 * Adjust the struct and bit layout below to match your HLS core's output format.
 */
typedef struct {
    uint16_t x;
    uint16_t y;
    int16_t  vx;
    int16_t  vy;
} flow_vector_t;

/*
 * Unpack a single 64-bit AXI-Stream word from the FPGA into a flow_vector_t.
 *
 * Bit layout (adjust to match your HLS core's actual output format):
 *   [63:48]  x   (uint16)
 *   [47:32]  y   (uint16)
 *   [31:16]  vx  (int16, signed Q8.8)
 *   [15:0]   vy  (int16, signed Q8.8)
 */
void flow_unpack(uint64_t word, flow_vector_t *fv);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_UNPACK_H */
