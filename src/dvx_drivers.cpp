
#include <libcaer/libcaer.h>
#include <libcaer/devices/dvxplorer.h>
#include <libcaer/events/polarity.h>
#include <opencv2/opencv.hpp>


#include <signal.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include "../include/dvx_drivers.hpp"
#include "../third_party/fast_detector.h"


// static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}



event_t get_events(caerPolarityEventPacket polarity, int j){
    event_t evnt;

    caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);
    if (!caerPolarityEventIsValid(evt)){
        exit(EXIT_FAILURE);
    }

    evnt.x = caerPolarityEventGetX(evt);
    evnt.y = caerPolarityEventGetY(evt);
    evnt.p = caerPolarityEventGetPolarity(evt);
    evnt.t = caerPolarityEventGetTimestamp64(evt, polarity);

    return evnt;
}

int main(void){
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
    
    cv::Mat canvas(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));

    corner_event_detector::FastDetector detector;
    while (!globalShutdown.load(std::memory_order_relaxed)) {
            canvas.setTo(cv::Scalar(128, 128, 128));
            caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);
            if (packetContainer == NULL) {
                // if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC
                continue;
            }

            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

            
            int radius = 1;
            int thickness = -1;

            for (int32_t i = 0; i < packetNum; i++) {
                caerEventPacketHeader packetHeader =
                    caerEventPacketContainerGetEventPacket(packetContainer, i);
                if (packetHeader == NULL) continue;
                if (caerEventPacketHeaderGetEventType(packetHeader) != POLARITY_EVENT) continue;

                caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
                int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

                for (int32_t j = 0; j < eventNum; j++) {
                    
                    event_t evnt = get_events(polarity, j);

                    bool feature = detector.isFeature(evnt);

                    if (feature == true){
                        cv::circle(canvas, cv::Point(evnt.x, evnt.y), radius, cv::Scalar(0, 0, 255), thickness);
                        // std::cout<< "feature position x: " << evnt.x << std::endl;
                        // std::cout<< "feature position y: " << evnt.y << std::endl;
                    }
                    
                }
                
            
            
    }
      caerEventPacketContainerFree(packetContainer);

    // fade the whole canvas toward black so old corners decay
    canvas *= 0.90;

    cv::imshow("Features", canvas);
    if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC to quit

    
}
caerDeviceDataStop(dvxplr_hndl);
    caerDeviceClose(&dvxplr_hndl);
    // cv::destroyAllWindows();
    printf("Shutdown successful.\n");
    return EXIT_SUCCESS;
}