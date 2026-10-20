#include "../include/DVX_Drivers.hpp"
#include "../third_party/fast_detector.h"
#include "../include/Feature.hpp"
#include "../include/OpticalFlow.hpp"
#include <algorithm>


// static std::atomic<bool> globalShutdown(false);
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}


// int trackFeatures(feature_t feat){
    
// }


Event get_events(caerPolarityEventPacket polarity, int j){
    Event evnt;

    caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);
    if (!caerPolarityEventIsValid(evt)){
        exit(EXIT_FAILURE);
    }

    //X and Y already come back as uint16_t, the same type PixelCoord stores,

    evnt.position<< caerPolarityEventGetX(evt), caerPolarityEventGetY(evt);
    evnt.polarity = caerPolarityEventGetPolarity(evt);
    // time stamp 64 is int64 microsecons; event keep uint32 (matches event_t)

    evnt.timestamp = static_cast<uint32_t>(caerPolarityEventGetTimestamp64(evt, polarity));


    return evnt;
}


ImuSample get_imu_sample(caerIMU6EventPacket imuPacket, int j){
    ImuSample sample;

    caerIMU6Event imuEvent = caerIMU6EventPacketGetEvent(imuPacket, j);
    if (!caerIMU6EventIsValid(imuEvent)){
        exit(EXIT_FAILURE);
    }

    sample.timestamp = (double) caerIMU6EventGetTimestamp64(imuEvent, imuPacket);
    sample.accel << (double) caerIMU6EventGetAccelX(imuEvent), (double) caerIMU6EventGetAccelY(imuEvent), (double) caerIMU6EventGetAccelZ(imuEvent);
    sample.gyro << (double) caerIMU6EventGetGyroX(imuEvent), (double) caerIMU6EventGetGyroY(imuEvent), (double) caerIMU6EventGetGyroZ(imuEvent);

    return sample;
}

RawData get_raw_data(caerDeviceHandle dvxplr_hndl){
    RawData data;

    caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);
    
    if (packetContainer == NULL) {
        return data;
    }

    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

    for (int32_t i = 0; i < packetNum; i++) {
        caerEventPacketHeader packetHeader =
            caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL) {
            continue;
        }
        //checking if IMU6 event


        int eventType = caerEventPacketHeaderGetEventType(packetHeader);

        if (eventType == POLARITY_EVENT){
            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;
            int32_t eventNum = caerEventPacketHeaderGetEventNumber(packetHeader);
            data.events.reserve(data.events.size() + eventNum);

            for (int32_t j = 0; j < eventNum; j++) {
                data.events.push_back(get_events(polarity, j));
            }
        }
        else if (eventType == IMU6_EVENT) {
            caerIMU6EventPacket imuPacket = (caerIMU6EventPacket) packetHeader;
            int32_t imuEventNum = caerEventPacketHeaderGetEventNumber(packetHeader);

            for (int32_t j = 0; j < imuEventNum; j++) {
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
    
    cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(128, 128, 128));
    
    double highestTimestamp = 0;
    std::vector<Event> eventBuffer; //time sorted only events some feature window need
    std::vector<Feature> currFeatures;
    currFeatures.reserve(MAXFEAT);
    corner_event_detector::FastDetector detector;
    const double epsilon = 0.5;
    int radius = 1;
    int thickness = -1;
    while (!globalShutdown.load(std::memory_order_relaxed)) {
            RawData data = get_raw_data(dvxplr_hndl);
            if (data.events.empty() && data.imu_samples.empty()) {
                continue;
            }

            for (const ImuSample& imu : data.imu_samples) {
                std::cout << "Accel: " << imu.accel.x() << "," << imu.accel.y() << "," << imu.accel.z()
                << "Gyro: " << imu.gyro.x() << "," << imu.gyro.y() << "," << imu.gyro.z()
                << '\n';
            }
            canvas.setTo(cv::Scalar(128, 128, 128));
            
            // detection stays on this thread: the SAE inside the dector is shared , per event stat


            for (const Event&e : data.events) {
                highestTimestamp = std::max(highestTimestamp, e.t());
                if (detector.isFeature(e)) continue;
                
                cv::circle(canvas, cv::Point(e.position.x(),e.position.y()), radius, cv::Scalar(255,255,255), thickness);

                Eigen::Vector2d newPos = e.pos();
                bool nearExisting = std::any_of(currFeatures.begin(), currFeatures.end(), [&](const Feature& f) {
                    return (newPos - f.getPosition()).norm() < RAD;
                });
                
                if (!nearExisting && (int) currFeatures.size() < MAXFEAT) {
                    currFeatures.push_back(Feature(newPos,e.t(), 0.0, {},{},Eigen::Vector2d(0,0),RAD));
                }
            }

            eventBuffer.insert(eventBuffer.end(), data.events.begin(), data.events.end());

            // each feature only reads the shared event vectors and writes itself so features run in parallel

            #pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < (int) currFeatures.size(); i++) {
                FeatureCollection(currFeatures[i], data.events, eventBuffer, epsilon, highestTimestamp);
            }
            

            currFeatures.erase(std::remove_if(currFeatures.begin(),currFeatures.end(), [](const Feature& f) {
                return f.getLife();
            }), currFeatures.end());

            //drop events older than every open window other wise buffer grow forver
            double oldestNeeded = highestTimestamp;
            for (const Feature& f : currFeatures) {
                if (f.checkWindow()) {
                    oldestNeeded = std::min(oldestNeeded, f.getTi());
                }
            }
            // assumes timestamp arrive in order

            eventBuffer.erase(eventBuffer.begin(), std::lower_bound(eventBuffer.begin(), eventBuffer.end(), oldestNeeded, [](const Event& e, double t) {
                return e.t() < t;
            }));

            // fade the whole canavs toward black so old corners decay

            //canavs *= 0.90;

            cv::imshow("Feature", canvas);
            if (cv::waitKey(1) == 27) { // Exit on ESC
                globalShutdown.store(true);
            }
    }

caerDeviceDataStop(dvxplr_hndl);
caerDeviceClose(&dvxplr_hndl);
// cv::destroyAllWindows();
printf("Shutdown successful.\n");
return EXIT_SUCCESS;
}
