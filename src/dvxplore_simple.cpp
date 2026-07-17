// dvx_events_only.cpp
// DVXplorer -> plain event visualization. Events are accumulated into a frame
// (ON=white, OFF=black on a gray background) and shown; the buffer clears once
// EVENTS_PER_FRAME events have piled up. No optical flow, no plane fitting.
//
// Build (Linux, adjust paths as needed):
//   g++ -O3 -std=c++17 dvx_events_only.cpp -o dvx_events_only \
//       $(pkg-config --cflags --libs libcaer opencv4)

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

// ------------------------------- config -----------------------------------
// Comment this out to show optical-flow lines only; leave it defined to show
// raw ON/OFF event pixels only.
// #define SHOW_RAW_EVENTS

static constexpr int FRAME_WIDTH  = 640;
static constexpr int FRAME_HEIGHT = 480;

const size_t KERNEL_SIZE = 3;
const size_t KERNEL_2SIZE = KERNEL_SIZE*KERNEL_SIZE;
// Redraw + clear the frame once this many events have accumulated.
// Smaller = higher refresh / sparser frames, larger = denser frames.
static constexpr int EVENTS_PER_FRAME = 20000;

// Plane-fit tuning. TAU rejects neighbor taps whose last event is older than
// this many microseconds relative to the center event, i.e. a stale echo of
// some earlier, unrelated edge rather than the same moving edge. The EPS_*
// values are just numerical-noise guards, not sensitivity knobs.
static constexpr int64_t TAU        = 20000;
static constexpr double  EPS_DET    = 1e-6;
static constexpr double  EPS_GRAD2  = 1e-9;

// Flow-vector overlay: line length ~ speed * this many microseconds of
// travel, capped in pixels so a near-singular fit (huge apparent speed)
// doesn't paint a line off the edge of the frame.
static constexpr double FLOW_ARROW_TIME_US  = 300.0;
static constexpr double FLOW_ARROW_MAX_LEN  = 40.0;
static constexpr double FLOW_ARROW_MAX_LEN2 = FLOW_ARROW_MAX_LEN * FLOW_ARROW_MAX_LEN;

// ---------------------------- global state --------------------------------
static std::atomic<bool> globalShutdown(false);

// ----------------------------- signal glue --------------------------------
static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) globalShutdown.store(true);
}
static void usbShutdownHandler(void *ptr) {
    (void) ptr;
    globalShutdown.store(true);
}

int64_t t_surface[FRAME_WIDTH][FRAME_HEIGHT] = {{0}};
bool pol_surface[FRAME_WIDTH][FRAME_HEIGHT] = {{0}};
int A[KERNEL_SIZE*KERNEL_SIZE][KERNEL_SIZE] = {
    {-1, -1, 1},   // k=0  (dx=-1, dy=-1)
    { 0, -1, 1},   // k=1
    { 1, -1, 1},   // k=2
    {-1,  0, 1},   // k=3
    { 0,  0, 1},   // k=4  center
    { 1,  0, 1},   // k=5
    {-1,  1, 1},   // k=6
    { 0,  1, 1},   // k=7
    { 1,  1, 1}
};

int A_T[KERNEL_SIZE][KERNEL_2SIZE] = {
    {-1, 0, 1, -1, 0, 1, -1, 0, 1},
    {-1, -1, -1, 0, 0, 0, 1, 1, 1},
    {1,1,1,1,1,1,1,1,1}
};






// -------------------------------- main ------------------------------------
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

    // --- Accumulation buffer: gray background, flow vectors drawn on top ---
    const cv::Scalar BACKGROUND(128, 128, 128);
    const cv::Scalar FLOW_COLOR(255, 255, 255);
    cv::Mat accumulator(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, BACKGROUND);
    int accumulatedEvents = 0;

    cv::namedWindow("DVXplorer Events", cv::WINDOW_AUTOSIZE);

    uint16_t kernel[KERNEL_SIZE][KERNEL_SIZE];
    int64_t t_temp[KERNEL_2SIZE];
    bool pol_temp[KERNEL_2SIZE];



    while (!globalShutdown.load(std::memory_order_relaxed)) {
        caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplr_hndl);
        if (packetContainer == NULL) {
            if (cv::waitKey(1) == 27) globalShutdown.store(true);  // ESC
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
                
                memset(t_temp, 0, sizeof(t_temp));
                memset(kernel, 0, sizeof(kernel));
                memset(pol_temp, 0, sizeof(pol_temp));


                caerPolarityEvent evt = caerPolarityEventPacketGetEvent(polarity, j);
                if (!caerPolarityEventIsValid(evt)) continue;

                uint16_t x = caerPolarityEventGetX(evt);
                uint16_t y = caerPolarityEventGetY(evt);
                bool     p = caerPolarityEventGetPolarity(evt);
                int64_t t = caerPolarityEventGetTimestamp64(evt, polarity);

                t_surface[x][y] = t;
                pol_surface[x][y] = p;

                if (x < 1 || y < 1 || x >= FRAME_WIDTH - 1 || y >= FRAME_HEIGHT - 1) continue;

#ifndef SHOW_RAW_EVENTS
                for (int i = 0; i < KERNEL_2SIZE; i++){
                    t_temp[i] = t_surface[A[i][0] + x][A[i][1] + y];
                    pol_temp[i] = pol_surface[A[i][0] + x][A[i][1] + y];
                }

                // ---- weighted least-squares plane fit over the 3x3 neighborhood ----
                // Model: t = a*dx + b*dy + c, solved from AtWA*[a b c]^T = AtWB.
                // Because A's entries are the fixed {-1,0,1} offsets and W is
                // diagonal (0/1 per tap), the matmul collapses into these nine
                // weighted scalar sums - no 9x3 / 9x9 matrices are built.
                double Sw = 0.0, Swdx = 0.0, Swdy = 0.0;
                double Swdx2 = 0.0, Swdxdy = 0.0, Swdy2 = 0.0;
                double Swdt = 0.0, Swdxdt = 0.0, Swdydt = 0.0;

                // A tap counts only if same polarity as the center event, has
                // actually fired (t_temp != 0, the surface's reset value), and
                // is not stale (within TAU of the center timestamp). Timestamps
                // are made relative to the center (dt) before use, since the
                // raw microsecond values are too large for stable float math.
#define PLANE_FIT_TAP(k, dx, dy)                                              \
                    do {                                                      \
                        bool hasFired  = (t_temp[k] != 0);                    \
                        int64_t stale  = t - t_temp[k];                       \
                        bool w = hasFired && (pol_temp[k] == p) &&            \
                                 (stale >= 0) && (stale <= TAU);              \
                        if (w) {                                             \
                            double dt = static_cast<double>(t_temp[k] - t);   \
                            Sw     += 1.0;                                    \
                            Swdx   += (dx);                                  \
                            Swdy   += (dy);                                  \
                            Swdx2  += (dx) * (dx);                           \
                            Swdxdy += (dx) * (dy);                          \
                            Swdy2  += (dy) * (dy);                           \
                            Swdt   += dt;                                    \
                            Swdxdt += (dx) * dt;                            \
                            Swdydt += (dy) * dt;                            \
                        }                                                    \
                    } while (0)

                // Unrolled over the fixed 3x3 offsets; order matches A's rows.
                PLANE_FIT_TAP(0, -1, -1);
                PLANE_FIT_TAP(1,  0, -1);
                PLANE_FIT_TAP(2,  1, -1);
                PLANE_FIT_TAP(3, -1,  0);
                PLANE_FIT_TAP(4,  0,  0);
                PLANE_FIT_TAP(5,  1,  0);
                PLANE_FIT_TAP(6, -1,  1);
                PLANE_FIT_TAP(7,  0,  1);
                PLANE_FIT_TAP(8,  1,  1);
#undef PLANE_FIT_TAP

                bool haveFlow = false;
                double vx = 0.0, vy = 0.0;

                // Need >= 3 independent taps to define a plane at all.
                if (Sw >= 3.0) {
                    // Symmetric normal matrix M and RHS B (c is unused for flow).
                    //   M = [ Swdx2   Swdxdy  Swdx ]   B = [ Swdxdt ]
                    //       [ Swdxdy  Swdy2   Swdy ]       [ Swdydt ]
                    //       [ Swdx    Swdy    Sw   ]       [ Swdt   ]
                    double m00 = Swdx2,  m01 = Swdxdy, m02 = Swdx;
                    double m10 = Swdxdy, m11 = Swdy2,  m12 = Swdy;
                    double m20 = Swdx,   m21 = Swdy,   m22 = Sw;
                    double b0 = Swdxdt, b1 = Swdydt, b2 = Swdt;

                    double det = m00 * (m11 * m22 - m12 * m21)
                               - m01 * (m10 * m22 - m12 * m20)
                               + m02 * (m10 * m21 - m11 * m20);

                    if (std::fabs(det) >= EPS_DET) {
                        // Cramer's rule, columns 0 and 1 only (a and b).
                        double detA = b0  * (m11 * m22 - m12 * m21)
                                    - m01 * (b1  * m22 - m12 * b2)
                                    + m02 * (b1  * m21 - m11 * b2);

                        double detB = m00 * (b1  * m22 - m12 * b2)
                                    - b0  * (m10 * m22 - m12 * m20)
                                    + m02 * (m10 * b2  - b1  * m20);

                        double a = detA / det;
                        double b = detB / det;

                        // Gradient (a,b) = (dt/dx, dt/dy) is "slowness" in
                        // s/px; flow is its inverse (normal flow only, along
                        // the gradient direction - the aperture problem means
                        // that's all a single edge patch can give us).
                        double gradMag2 = a * a + b * b;
                        if (gradMag2 >= EPS_GRAD2) {
                            vx = a / gradMag2;
                            vy = b / gradMag2;
                            haveFlow = true;
                        }
                    }
                }
#endif // !SHOW_RAW_EVENTS

#ifdef SHOW_RAW_EVENTS
                // Raw event pixel: ON -> white, OFF -> black.
                accumulator.at<cv::Vec3b>(y, x) =
                    p ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
#else
                // --- flow-vector overlay: draw directly on the same accumulator
                // frame the event visualizer uses, so both share one image.
                // Plain thin line, no arrowhead, no antialiasing, no color
                // computation, and sqrt is skipped unless the vector is long
                // enough to need capping - kept minimal so a burst of events
                // during sharp motion doesn't stall the redraw. ---
                if (haveFlow) {
                    double dxLine = vx * FLOW_ARROW_TIME_US;
                    double dyLine = vy * FLOW_ARROW_TIME_US;
                    double len2   = dxLine * dxLine + dyLine * dyLine;
                    if (len2 > FLOW_ARROW_MAX_LEN2) {
                        double s = FLOW_ARROW_MAX_LEN / std::sqrt(len2);
                        dxLine *= s;
                        dyLine *= s;
                    }

                    cv::Point tip(cvRound(x + dxLine), cvRound(y + dyLine));
                    cv::line(accumulator, cv::Point(x, y), tip, FLOW_COLOR, 1, cv::LINE_8);
                }
#endif

                if (x >= FRAME_WIDTH || y >= FRAME_HEIGHT) continue;

                // periodically show + clear
                if (++accumulatedEvents >= EVENTS_PER_FRAME) {
                    cv::imshow("DVXplorer Events", accumulator);
                    if (cv::waitKey(1) == 27) globalShutdown.store(true);
                    accumulator.setTo(BACKGROUND);
                    accumulatedEvents = 0;
                }
            }
        }
        caerEventPacketContainerFree(packetContainer);
    }

    caerDeviceDataStop(dvxplr_hndl);
    caerDeviceClose(&dvxplr_hndl);
    cv::destroyAllWindows();
    printf("Shutdown successful.\n");
    return EXIT_SUCCESS;
}