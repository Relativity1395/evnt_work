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

typedef struct{
    uint16_t x;
    uint16_t y;
    uint32_t t;
    bool p;
}event_t;

static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal);
int get_events(void);

#endif