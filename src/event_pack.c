#include "event_pack.h"

uint64_t event_pack(int32_t timestamp, uint16_t x, uint16_t y, bool polarity) {
    uint64_t word = 0;
    word |= ((uint64_t)(uint32_t)timestamp)      << 32;
    word |= ((uint64_t)(x & 0xFFFu))             << 20;
    word |= ((uint64_t)(y & 0xFFFu))             <<  8;
    word |= ((uint64_t)(polarity ? 1u : 0u))     <<  7;
    return word;
}
