#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* A single polarity event from the DVXplorer */
typedef struct {
    int32_t  timestamp;   /* microseconds */
    uint16_t x;
    uint16_t y;
    bool     polarity;    /* true = ON (+), false = OFF (-) */
} dvs_event_t;

/*
 * Optical flow vector output from the FPGA.
 *
 * vx and vy are signed Q8.8 fixed-point (divide by 256 to get float pixels/s).
 * Adjust the format below if your HLS core uses a different encoding.
 *
 * 64-bit word layout (from flow_out AXI-Stream):
 *   [63:48] x  (uint16)
 *   [47:32] y  (uint16)
 *   [31:16] vx (int16, Q8.8)
 *   [15:0]  vy (int16, Q8.8)
 */
typedef struct {
    uint16_t x;
    uint16_t y;
    int16_t  vx;
    int16_t  vy;
} flow_vector_t;

#endif /* TYPES_H */
