// Benchmark: how many features can be tracked in real time, single-threaded vs multithreaded.
// Synthetic L-corners move at a known flow; each one closes a window and runs a full EM pass,
// exactly like FeatureCollection does on the camera. No camera needed.
#include "../include/Feature.hpp"
#include "../include/OpticalFlow.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <omp.h>

int main(){
    const Eigen::Vector2d u(4e-4, 2e-4);            // 400, 200 px/s
    const double T0 = 1e6, dt = 6700.0;             // ~3 px of motion per window, like the adaptive window
    const double T1 = T0 + dt, T2 = T1 + dt;
    const int N = 100;                              // features in the benchmark (10 x 10 grid)

    std::vector<Event> buffer;
    std::vector<std::vector<Event>> prev(N);
    std::vector<Eigen::Vector2d> corner(N);
    std::vector<PixelCoord> lastPix(N * 13, PixelCoord(0, 0));
    for (int f = 0; f < N; f++) corner[f] = Eigen::Vector2d(20 + (f % 10) * 60, 20 + (f / 10) * 45);

    for (double t = T0; t <= T2; t += 100.0) {
        for (int f = 0; f < N; f++) {
            for (int a = 0; a < 13; a++) {
                Eigen::Vector2d p = corner[f] + (a < 7 ? Eigen::Vector2d(a, 0) : Eigen::Vector2d(0, a - 6)) + (t - T0) * u;
                Event e;
                e.position << (uint16_t)std::lround(p.x()), (uint16_t)std::lround(p.y());
                e.timestamp = (uint32_t)t;
                e.polarity = true;
                if (lastPix[f * 13 + a] == e.position) continue;   // a pixel fires once as the edge crosses it
                lastPix[f * 13 + a] = e.position;
                buffer.push_back(e);
                if (t < T1) prev[f].push_back(e);
            }
        }
    }

    const int maxThreads = omp_get_max_threads();
    std::cout << "CPU threads available: " << maxThreads << "\n"
              << "features: " << N << ", events/feature/window: " << prev[0].size()
              << ", window length: " << dt / 1000.0 << " ms\n\n";

    std::vector<int> threadCounts;                  // 1, 2, 4, ... and always exactly maxThreads last
    for (int t = 1; t < maxThreads; t *= 2) threadCounts.push_back(t);
    threadCounts.push_back(maxThreads);

    double singleUs = 0.0;
    for (int threads : threadCounts) {
        double best = 1e300;
        for (int rep = 0; rep < 5; rep++) {             // best of 5 to skip warm-up / scheduler noise
            std::vector<Feature> F;
            F.reserve(N);
            for (int f = 0; f < N; f++) F.push_back(Feature(corner[f] + (T1 - T0) * u, T1, dt, {}, prev[f], u, 15.0));

            omp_set_num_threads(threads);
            auto start = std::chrono::steady_clock::now();
            #pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < N; i++) FeatureCollection(F[i], {}, buffer, 0.5, T2);
            best = std::min(best, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());
        }
        if (threads == 1) singleUs = best;
        // real time = all features finish their window before the next window's worth of data arrives
        const int capacity = (int)(dt / (best / N));
        std::cout << threads << " thread(s): " << best / 1000.0 << " ms for " << N << " feature windows"
                  << " | speedup " << singleUs / best << "x"
                  << " | ~" << capacity << " features in real time"
                  << " (~" << capacity / threads << " per thread)\n";
    }
    return 0;
}