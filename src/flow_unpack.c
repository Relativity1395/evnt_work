#include "flow_unpack.h"

void flow_unpack(uint64_t word, flow_vector_t *fv) {
    fv->x  = (uint16_t)((word >> 48) & 0xFFFFu);
    fv->y  = (uint16_t)((word >> 32) & 0xFFFFu);
    fv->vx = (int16_t) ((word >> 16) & 0xFFFFu);
    fv->vy = (int16_t) ((word >>  0) & 0xFFFFu);
}
