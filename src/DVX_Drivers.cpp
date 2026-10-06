#include "../include/DVX_Drivers.hpp"
#include "../third_party/fast_detector.h"
#include "../include/Feature.hpp"

// static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}

Event get_event(caerPolarityEventPacket polarity, int j) {
    Event evnt;

    caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);

    if (!caerPolarityEventIsValid(evt)) {
        exit(EXIT_FAILURE);
    }
    //X and Y already come back as uint16_t, the same type PixelCoord stores, 
    evnt.position << caerPolarityEventGetX(evt), caerPolarityEventGetY(evt);
    evnt.polarity = caerPolarityEventGetPolarity(evt);
    // time stamp 64 is int64 microsecons; event keep uint32 (matches event_t)
    //which wraps after 71.6 minutes of camera uptime. So we use the 64 bit version to avoid wrap around issues.
    evnt.timestamp = static_cast<uint32_t>(caerPolarityEventGetTimestamp64(evt, polarity));

    return evnt;
}


ImuSample get_imu_sample(caerIMU6EventPacket imuPacket, int j) {
    ImuSample sample;

    caerIMU6Event imuEvent = caerIMU6EventPacketGetEvent(imuPacket, j);

    if (!caerIMU6EventIsValid(imuEvent)) {
        exit(EXIT_FAILURE);
    }

    sample.timestamp = (double) caerIMU6EventGetTimestamp64(imuEvent, imuPacket);
    sample.accel << (double) caerIMU6EventGetAccelX(imuEvent), (double) caerIMU6EventGetAccelY(imuEvent), (double) caerIMU6EventGetAccelZ(imuEvent);
    sample.gyro << (double) caerIMU6EventGetGyroX(imuEvent), (double) caerIMU6EventGetGyroY(imuEvent), (double) caerIMU6EventGetGyroZ(imuEvent);

    return sample;
}


RawData get_raw_data(caerDeviceHandle dvxplr_hndl) {
    RawData data;
    caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);

    if (packetContainer == NULL) {
        return data;
    }

    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

    for (int32_t i = 0; i < packetNum; i++) {
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);

        if (packetHeader == NULL) {
            continue;
        }

        int eventType = caerEventPacketHeaderGetEventType(packetHeader);

        if (eventType == POLARITY_EVENT) {
            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
            int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

            for (int32_t j = 0; j < eventNum; j++) {
                data.events.push_back(get_event(polarity, j));
            }
        }

        else if (eventType == IMU6_EVENT) {
            caerIMU6EventPacket imuPacket = (caerIMU6EventPacket) packetHeader;
            int32_t imuNum = caerEventPacketHeaderGetEventNumber(packetHeader);

            for (int32_t j = 0; j < imuNum; j++) {
                data.imu_samples.push_back(get_imu_sample(imuPacket, j));
            }
        }
    }

    caerEventPacketContainerFree(packetContainer);

    return data;
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
    double highestTimestamp = 0;
    std::vector<Event> currEvents;
    std::vector<Feature> currFeatures;
    std::vector<Feature> tempFeatures;
    corner_event_detector::FastDetector detector;
    const double epsilon = 0.5;
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
                //checking if IMU6 event
                if (caerEventPacketHeaderGetEventType(packetHeader) == IMU6_EVENT){
                    caerIMU6EventPacket imuPacket = caerIMU6EventPacketFromPacketHeader(packetHeader);
                    int32_t imuEventNum = caerEventPacketHeaderGetEventNumber(packetHeader);
                    for(int32_t j = 0; j < imuEventNum; j++){
                        //getting each event one by one in the packet
                        caerIMU6Event imuEvent = caerIMU6EventPacketGetEvent(imuPacket, j);

                        if(!caerIMU6EventIsValid(imuEvent)){
                            continue;
                        }
                        imu_t imu;
                        imu.t = caerIMU6EventGetTimestamp64(imuEvent, imuPacket);

                        //setting accel values
                        imu.accel_x = caerIMU6EventGetAccelX(imuEvent);
                        imu.accel_y = caerIMU6EventGetAccelY(imuEvent);
                        imu.accel_z = caerIMU6EventGetAccelZ(imuEvent);

                        //setting gyro values
                        imu.gyro_x = caerIMU6EventGetGyroX(imuEvent);
                        imu.gyro_y = caerIMU6EventGetGyroY(imuEvent);
                        imu.gyro_z = caerIMU6EventGetGyroZ(imuEvent);

                        imu.temperature = caerIMU6EventGetTemp(imuEvent);
                        //printing the event 
                        std::cout
                        << "ACCEL: "
                        << imu.accel_x << ","
                        << imu.accel_y << ","
                        << imu.accel_z
                        << " | GYRO: "
                        << imu.gyro_x << ","
                        << imu.gyro_y << ","
                        << imu.gyro_z 
                        << '\n' ;


                    }
                }
                    
                    
    
                if (caerEventPacketHeaderGetEventType(packetHeader) == POLARITY_EVENT) {
                    caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
                    int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);
                    tempFeatures.clear();
                    for (int32_t j = 0; j < eventNum; j++) {
                        
                        
                        currEvents.push_back(get_event(polarity, j));
                        if (currEvents.back().timestamp > highestTimestamp){
                            highestTimestamp = currEvents.back().timestamp;
                        }
                        bool feature = detector.isFeature(currEvents.back());
                        //test push main
                        if (feature == true){
                            for (int i = 0; i < (int)currFeatures.size(); i++){
                                if (currEvents.back().position.norm() <= currFeatures[i].getPosition().norm() + 1){
                                    if ((i == ((int)currFeatures.size() - 1)) && currFeatures.size() < 100){
                                        Feature Feature(currEvents.back().pos(), currEvents.back().t(), 0.0, {}, {}, Eigen::Vector2d(0,0), 15.0);
                                        currFeatures.push_back(Feature);
                                        break;
                                    }
                                    continue;

                                }
                                break;
                            }
                            
                            
                            cv::circle(canvas, cv::Point(currEvents.back().position.x(), currEvents.back().position.y()), radius, cv::Scalar(255, 255, 255), thickness);
                            // std::cout<< "feature position x: " << evnt.x << std::endl;
                            // std::cout<< "feature position y: " << evnt.y << std::endl;
                        }
                        // else if (evnt.p == 1){
                        // cv::circle(canvas, cv::Point(evnt.x, evnt.y), radius, cv::Scalar(255, 255, 255), thickness);
                        // }else{
                        //     cv::circle(canvas, cv::Point(evnt.x, evnt.y), radius, cv::Scalar(0, 0, 0), thickness);
                        // }
                        
                }





            }
            
            


                
            
            
    }

    
    for (int i = 0; i < (int)currFeatures.size(); i++){
        currFeatures[i].findEvents(currEvents);
        currFeatures[i].propagatePreviousEvents();
        
        if (currFeatures[i].checkWindow()){
            while (currFeatures[i].calculateCost() > epsilon){
                currFeatures[i].generateKD();
                currFeatures[i].updateFlow();
            }
            
            currFeatures[i].checkHealth();
            if (currFeatures[i].getDeadCycles() == 3){
                currFeatures.erase(currFeatures.begin() + i);
                continue;
            }
        }
        if (currFeatures[i].checkInitWindow()){
            currFeatures[i].updateTimeWindow(highestTimestamp);
            currFeatures[i].setEvents();
        }
    }
    caerEventPacketContainerFree(packetContainer);

    // fade the whole canvas toward black so old corners decay
    // canvas *= 0.90;

    cv::imshow("Features", canvas);
    if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC to quit

    
}

caerDeviceDataStop(dvxplr_hndl);
    caerDeviceClose(&dvxplr_hndl);
    // cv::destroyAllWindows();
    printf("Shutdown successful.\n");
    return EXIT_SUCCESS;
}