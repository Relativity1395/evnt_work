#include "../include/DVX_Drivers.hpp"
#include "../third_party/fast_detector.h"
#include "../include/Feature.hpp"
#include "../include/OpticalFlow.hpp"

#include <fstream>
#include <iomanip>

// static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}

bool get_event(caerPolarityEventPacket polarity, int j, Event& evnt) {
    caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);
    if (!caerPolarityEventIsValid(evt)) return false;
    evnt.position << caerPolarityEventGetX(evt), caerPolarityEventGetY(evt);
    evnt.polarity  = caerPolarityEventGetPolarity(evt);
    evnt.timestamp = static_cast<uint32_t>(caerPolarityEventGetTimestamp64(evt, polarity));
    return true;
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


// RawData get_raw_data(caerDeviceHandle dvxplr_hndl) {
//     RawData data;
//     caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);

//     if (packetContainer == NULL) {
//         return data;
//     }

//     int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

//     for (int32_t i = 0; i < packetNum; i++) {
//         caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);

//         if (packetHeader == NULL) {
//             continue;
//         }

//         int eventType = caerEventPacketHeaderGetEventType(packetHeader);

//         if (eventType == POLARITY_EVENT) {
//             caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
//             int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

//             for (int32_t j = 0; j < eventNum; j++) {
//                 data.events.push_back(get_event(polarity, j));
//             }
//         }

//         else if (eventType == IMU6_EVENT) {
//             caerIMU6EventPacket imuPacket = (caerIMU6EventPacket) packetHeader;
//             int32_t imuNum = caerEventPacketHeaderGetEventNumber(packetHeader);

//             for (int32_t j = 0; j < imuNum; j++) {
//                 data.imu_samples.push_back(get_imu_sample(imuPacket, j));
//             }
//         }
//     }

//     caerEventPacketContainerFree(packetContainer);

//     return data;
// }


void logFeatureStatus(std::ofstream& logFile, const Feature& feature, double timestamp, size_t activeFeatures, int status, double wallMs, double batchMs) {
    const char* statusStr;

    if (status == DEAD)
        statusStr = "DEAD";
    else if (status == INIT)
        statusStr = "INIT";
    else
        statusStr = "ALIVE";

    logFile << feature.getID() << ","
        << std::fixed << std::setprecision(0) << timestamp << ","
        << statusStr << ","
        << std::setprecision(6)
        << feature.getPosition().x() << "," << feature.getPosition().y() << ","
        << feature.getCost() << ","
        << std::setprecision(8)
        << feature.getFlow().x() << "," << feature.getFlow().y() << ","
        << activeFeatures << ","
        << feature.getEventCount() << ","
        << (feature.hasConverged() ? "true" : "false") << ","
        << feature.getWindowSize() << "," 
        << feature.getIters() << "\n"
        << std::setprecision(3) << wallMs << "," << batchMs << "\n";


}
int main(void) {

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
    const bool IMU_CALIBRATION = false;
    const bool SHOW_WINDOW = false;
    // Open DVXplorer
    caerDeviceHandle dvxplr_hndl = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, NULL);
    if (dvxplr_hndl == NULL) return EXIT_FAILURE;
    caerDeviceSendDefaultConfig(dvxplr_hndl);
    // Set accelerometer to 400 Hz
    bool accelOK = caerDeviceConfigSet(
        dvxplr_hndl,
        DVX_IMU,
        DVX_IMU_ACCEL_DATA_RATE,
        5
    );

    // Set gyroscope to 400 Hz
    bool gyroOK = caerDeviceConfigSet(
        dvxplr_hndl,
        DVX_IMU,
        DVX_IMU_GYRO_DATA_RATE,
        4
    );

    std::cout << "Accelerometer configured: " << accelOK << "\n";
    std::cout << "Gyroscope configured: " << gyroOK << "\n";

    // Set packet interval to 5000 microseconds
    bool success = caerDeviceConfigSet(
        dvxplr_hndl,
        CAER_HOST_CONFIG_PACKETS,
        CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL,
        5000
    );

    // Read the interval to confirm
    uint32_t interval = 0;

    caerDeviceConfigGet(
        dvxplr_hndl,
        CAER_HOST_CONFIG_PACKETS,
        CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL,
        &interval
    );

    std::cout << "Configuration successful: " << success << "\n";
    std::cout << "Packet interval: " << interval << " us\n";

    caerDeviceDataStart(dvxplr_hndl, NULL, NULL, NULL, &usbShutdownHandler, NULL);
    // Disable DVS events during IMU calibration
    bool dvsOff = caerDeviceConfigSet(
        dvxplr_hndl,
        DVX_DVS,
        DVX_DVS_RUN,
        true
    );

    struct caer_dvx_info dvxplr_info = caerDVXplorerInfoGet(dvxplr_hndl);

    caerFilterDVSNoise noiseFilter = caerFilterDVSNoiseInitialize(dvxplr_info.dvsSizeX, dvxplr_info.dvsSizeY);
    caerFilterDVSNoiseConfigSet(noiseFilter, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE, true);
    caerFilterDVSNoiseConfigSet(noiseFilter, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME, 2000);  // µs
    caerFilterDVSNoiseConfigSet(noiseFilter, CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE, true);
    caerFilterDVSNoiseConfigSet(noiseFilter, CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME, 100);     // µs

    EventGrid grid(dvxplr_info.dvsSizeX, dvxplr_info.dvsSizeY);
    /*printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Firmware: %d.\n",
           dvxplr_info.deviceString, dvxplr_info.deviceID,
           dvxplr_info.deviceIsMaster, dvxplr_info.dvsSizeX,
           dvxplr_info.dvsSizeY, dvxplr_info.firmwareVersion);
*/
    
    caerDeviceConfigSet(dvxplr_hndl, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    
    cv::Mat canvas(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    double highestTimestamp = 0;
    std::vector<Event> currEvents;
    std::vector<Feature> currFeatures;
    std::vector<Feature> tempFeatures;
    corner_event_detector::FastDetector detector;
    const double epsilon = 5;
    const double minFeatureDist = 5.0;
    // before the while loop
    size_t detCount = 0, evCount = 0;

    size_t nextFeatureID = 0;
    double lastLogTime = 0;
    const double LOG_INTERVAL = 100;
    std::ofstream logFile("feature_status.csv");
    std::ofstream imuFile("imu_data400Hz.csv");
    if (!imuFile.is_open()) {
        std::cerr << "Failed to open IMU file!\n";
        return 1;
}

    imuFile << "timestamp,accel_x,accel_y,accel_z,"
            "gyro_x,gyro_y,gyro_z,temperature\n";

    imuFile << std::setprecision(10);
    

  

    logFile << "id,timestamp,status,px,py,cost,ux,uy,active,events,converged,window_size,iters,wall_ms,batch_ms\n" << "\n";

    if (!logFile.is_open()) {
        std::cerr << "Failed to open feature log file!\n";
        return 1;
    }
    
    auto lastDraw = std::chrono::steady_clock::now();
    const auto runStart = lastDraw;
    int64_t previousTimestamp = -1;

    int duplicateInside = 0;
    int duplicateBetween = 0;

    int gapInside = 0;
    int gapBetween = 0;
    while (!globalShutdown.load(std::memory_order_relaxed)) {
            int batchEvents = 0;
            int batchCorners = 0;
            if(!IMU_CALIBRATION){

            canvas.setTo(cv::Scalar(128, 128, 128));
            }
            if (std::chrono::steady_clock::now() - runStart
                >= std::chrono::seconds(60)) {
                break;
            }
            caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);
            if (packetContainer == NULL) {
                // if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC
                continue;
            }
            const auto batchStart = std::chrono::steady_clock::now();

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
                    static int packetCount = 0;

                    if (packetCount < 20) {
                        std::cout << "IMU packet " << packetCount
                                << " contains " << imuEventNum
                                << " events\n";
                    }

                    packetCount++;
                    for(int32_t j = 0; j < imuEventNum; j++){
                        //getting each event one by one in the packet
                        caerIMU6Event imuEvent = caerIMU6EventPacketGetEvent(imuPacket, j);

                        if(!caerIMU6EventIsValid(imuEvent)){
                            continue;
                        }
                        imu_t imu;
                        imu.t = caerIMU6EventGetTimestamp64(imuEvent, imuPacket);
                        if (previousTimestamp >= 0) {
                        int64_t dt = imu.t - previousTimestamp;
                        static int printed = 0;

                        if ((dt == 0 || dt > 5000) && printed < 30) {
                            std::cout << "Event index: " << j
                                    << " / " << imuEventNum
                                    << " | dt: " << dt
                                    << " us"
                                    << " | timestamp: " << imu.t
                                    << "\n";

                            printed++;
                        }
                        if (dt == 0) {
                            if (j == 0)
                                duplicateBetween++;
                            else
                                duplicateInside++;
                        }

                        if (dt > 5000) {
                            if (j == 0)
                                gapBetween++;
                            else
                                gapInside++;
                        }
                    }

                    previousTimestamp = imu.t;

                        //setting accel values
                        imu.accel_x = caerIMU6EventGetAccelX(imuEvent);
                        imu.accel_y = caerIMU6EventGetAccelY(imuEvent);
                        imu.accel_z = caerIMU6EventGetAccelZ(imuEvent);

                        //setting gyro values
                        imu.gyro_x = caerIMU6EventGetGyroX(imuEvent);
                        imu.gyro_y = caerIMU6EventGetGyroY(imuEvent);
                        imu.gyro_z = caerIMU6EventGetGyroZ(imuEvent);

                        imu.temperature = caerIMU6EventGetTemp(imuEvent);
                        imuFile << imu.t << ","
                        << imu.accel_x << ","
                        << imu.accel_y << ","
                        << imu.accel_z << ","
                        << imu.gyro_x << ","
                        << imu.gyro_y << ","
                        << imu.gyro_z << ","
                        << imu.temperature << "\n";
                        //printing the event 
                        // std::cout
                        // << "ACCEL: "
                        // << imu.accel_x << ","
                        // << imu.accel_y << ","
                        // << imu.accel_z
                        // << " | GYRO: "
                        // << imu.gyro_x << ","
                        // << imu.gyro_y << ","
                        // << imu.gyro_z 
                        // << '\n' ;


                    }
                }
                    
                    
    
    if (caerEventPacketHeaderGetEventType(packetHeader) == POLARITY_EVENT && !IMU_CALIBRATION) {
    caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
    caerFilterDVSNoiseApply(noiseFilter, polarity);
    int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

    for (int32_t j = 0; j < eventNum; j++) {
        Event e;
        if (!get_event(polarity, j, e)) continue;   // skips events the noise filter removed
        currEvents.push_back(e);
        Event& ev = currEvents.back();

        if (ev.timestamp > highestTimestamp) highestTimestamp = ev.timestamp;

        bool feature = detector.isFeature(ev);
        batchEvents++;
        evCount++;
        if (!feature) continue;
        batchCorners++;
        detCount++;

        bool isNew = true;
        for (const Feature& f : currFeatures) {
            if ((ev.pos() - f.getPosition()).norm() < minFeatureDist) { isNew = false; break; }
        }
        if (isNew && currFeatures.size() < MAXFEAT) {
            currFeatures.emplace_back(ev.pos(), ev.t(), 0.0, std::vector<Event>{},
                                      std::vector<Event>{}, Eigen::Vector2d(0, 0), 15.0);
            currFeatures.back().setID(nextFeatureID++);
        }

        cv::circle(canvas, cv::Point(ev.position.x(), ev.position.y()),
                   radius, cv::Scalar(255, 255, 255), thickness);
    }
}
                }
            
            
            // std::cout << "Features detected: " << currFeatures.size() << '\n';
            // std::cout << "Batch events: " << batchEvents
            //     << " | eFAST corners: " << batchCorners
            //     << " | Active features: " << currFeatures.size()
            //     << std::endl;
            if(!IMU_CALIBRATION){
                bool shouldLog = (highestTimestamp - lastLogTime >= LOG_INTERVAL);
            
                grid.build(currEvents);

                #pragma omp parallel for schedule(dynamic, 1)
                for (int i = 0; i < static_cast<int>(currFeatures.size()); i++) {
                    FeatureCollection(currFeatures[i], grid, currEvents, epsilon, highestTimestamp);
                    // std::cout << "thread number: " << omp_get_thread_num() << std::endl;
        
                }
            
                //std::cout << "FeatureCollection returned: " << result << '\n';
            const auto batchEnd = std::chrono::steady_clock::now();
            const double batchMs = std::chrono::duration<double, std::milli>(batchEnd - batchStart).count();
            const double wallMs = std::chrono::duration<double, std::milli>(batchEnd - runStart).count();
            for (int i = 0; i < static_cast<int>(currFeatures.size()); i++) {
                if (shouldLog || currFeatures[i].getHealth() == DEAD) {
                    // a feature about to be remove logs as dead otherwise init or alive
                    int status = currFeatures[i].getHealth() ? DEAD : currFeatures[i].getState();
                    logFeatureStatus(logFile, currFeatures[i], highestTimestamp, currFeatures.size(), status, wallMs, batchMs);
                }
                // std::cout << currFeatures[i].getWindowSize() << std::endl;
                // if (currFeatures[i].getFlow().norm() > 1e-4){
                //    std::cout << "ID: " << currFeatures[i].getID()<< " x position: " << currFeatures[i].getPosition().x() << " y position: " << currFeatures[i].getPosition().y() << " NumDeadCycles: " << currFeatures[i].getDeadCycles() << " COST: " << currFeatures[i].getCost() << " Window Size: " << currFeatures[i].getWindowSize() << std::endl;
                // }
                    if (currFeatures[i].getHealth() == DEAD) {

                    // std::cout << "feature died" << std::endl;
                    currFeatures.erase(currFeatures.begin() + i);
                    i--;
                    continue;
                }


                Eigen::Vector2d pos = currFeatures[i].getPosition();

                cv::circle(
                    canvas,
                    cv::Point(
                        static_cast<int>(pos.x()),
                        static_cast<int>(pos.y())
                    ),
                    3,
                    cv::Scalar(0, 255, 0),
                    1
                );
                //std::cout << "Flow: "
                //<< currFeatures[i].getFlow().transpose()
                //<< '\n';
                // next to your "Features detected" print
                
            }

            if (shouldLog) {
                lastLogTime = highestTimestamp;
            }
            
            // std::cout << "events: " << evCount << "  detector true: " << detCount
            // << "  MAXFEAT: " << MAXFEAT << '\n';    
            // caerEventPacketContainerFree(packetContainer);

            // cv::imshow("Features", canvas);
            

// Draw all stored features as green circles
            // for (Feature& f : currFeatures) {

                
            // }

            // // Display camera output
            if(!IMU_CALIBRATION && SHOW_WINDOW){    
                auto now = std::chrono::steady_clock::now();
                if (now - lastDraw >= std::chrono::milliseconds(33)) {
                    cv::imshow("Features", canvas);
                    cv::waitKey(1);
                    canvas.setTo(cv::Scalar(128, 128, 128));
                    lastDraw = now;
                }    
            }      
            
        }     
        caerEventPacketContainerFree(packetContainer);
        currEvents.clear();

    // std::vector<int> temp;
    // for (int i = 0; i < (int)currFeatures.size(); i++){
    //     if (FeatureCollection(currFeatures[i], currEvents, epsilon, highestTimestamp) != 0){
    //         std::cout << "IRan";
    //         temp[i] = 1;
    //     }
        
    // }

    // for (int i = 0; i < (int)temp.size(); i++){
    //     currFeatures.erase(currFeatures.begin() + i);
    // }
    


    // fade the whole canvas toward black so old corners decay
    // canvas *= 0.90;

    }
    std::cout << "Duplicates inside packets: "
          << duplicateInside << "\n";

    std::cout << "Duplicates between packets: "
            << duplicateBetween << "\n";

    std::cout << "Large gaps inside packets: "
            << gapInside << "\n";

    std::cout << "Large gaps between packets: "
            << gapBetween << "\n";
    caerDeviceDataStop(dvxplr_hndl);
    caerFilterDVSNoiseDestroy(noiseFilter);
    caerDeviceClose(&dvxplr_hndl);
    imuFile.close();
    // cv::destroyAllWindows();
    printf("Shutdown successful.\n");
    return EXIT_SUCCESS;

}
