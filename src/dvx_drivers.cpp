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
#include "dvx_drivers.hpp"

// static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}

int get_events(void){
#if defined(_WIN32)
    if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) return EXIT_FAILURE;
    if (signal(SIGINT,  &globalShutdownSignalHandler) == SIG_ERR) return EXIT_FAILURE;
#else
    struct sigaction shutdownAction;
    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags   = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);
    if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) return EXIT_FAILURE;
    if (sigaction(SIGINT,  &shutdownAction, NULL) == -1) return EXIT_FAILURE;
#endif

    // Open DVXplorer
    caerDeviceHandle dvxplr_hndl = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, NULL);
    if (dvxplr_hndl == NULL) return EXIT_FAILURE;

    struct caer_dvx_info dvxplr_info = caerDVXplorerInfoGet(dvxplr_hndl);
    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Firmware: %d.\n",
           dvxplr_info.deviceString, dvxplr_info.deviceID,
           dvxplr_info.deviceIsMaster, dvxplr_info.dvsSizeX,
           dvxplr_info.dvsSizeY, dvxplr_info.firmwareVersion);

    caerDeviceSendDefaultConfig(dvxplr_hndl);
    caerDeviceDataStart(dvxplr_hndl, NULL, NULL, NULL, &usbShutdownHandler, NULL);
    caerDeviceConfigSet(dvxplr_hndl, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    while (!globalShutdown.load(std::memory_order_relaxed)) {
            caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);
            if (packetContainer == NULL) {
                // if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC
                continue;
            }

            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

            for (int32_t i = 0; i < packetNum; i++) {
                caerEventPacketHeader packetHeader =
                    caerEventPacketContainerGetEventPacket(packetContainer, i);
                if (packetHeader == NULL) continue;
                if (caerEventPacketHeaderGetEventType(packetHeader) != POLARITY_EVENT) continue;

                caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
                int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

                for (int32_t j = 0; j < eventNum; j++) {
                    
                    event_t evnt;
                    // memset(t_temp, 0, sizeof(t_temp));
                    // memset(kernel, 0, sizeof(kernel));
                    // memset(pol_temp, 0, sizeof(pol_temp));


                    caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);
                    if (!caerPolarityEventIsValid(evt)) continue;

                    evnt.x = caerPolarityEventGetX(evt);
                    evnt.y = caerPolarityEventGetY(evt);
                    evnt.p = caerPolarityEventGetPolarity(evt);
                    evnt.t = caerPolarityEventGetTimestamp64(evt, polarity);

                    std::cout << "x position of pixel: " << evnt.x << std::endl;;
                    std::cout << "y position of pixel: " << evnt.y << std::endl;;
                }

            caerEventPacketContainerFree(packetContainer);
    }

    
}
caerDeviceDataStop(dvxplr_hndl);
    caerDeviceClose(&dvxplr_hndl);
    // cv::destroyAllWindows();
    printf("Shutdown successful.\n");
    return EXIT_SUCCESS;
}