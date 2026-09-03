#include <libcaer/libcaer.h>
#include <libcaer/devices/dvxplorer.h>
#include <libcaer/events/polarity.h>

#include <signal.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>

#ifndef DVX_DRIVERS_H
#define DVX_DRIVERS_H
#define WIDTH 640
#define HEIGHT 480

// uint16_t **sae = (uint16_t**)malloc((WIDTH*HEIGHT)*sizeof(uint16_t*));


typedef struct{
    uint16_t x;
    uint16_t y;
    uint32_t t;
    bool p;
}event_t;

typedef struct {
    uint16_t x;
    uint16_t y;
} coord_t;


static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal);
int get_events(void);

#endif