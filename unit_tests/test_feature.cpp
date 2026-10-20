#include "../include/Feature.hpp"
#include "../include/OpticalFlow.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>


//Build an event the way the driver does : integer pixels, microsecond timestamp.
static Event makeEvent(uint16_t x, uint16_t y, uint32_t t_us, bool polarity =true){
    Event e;
    e.position << x, y;
    e.timestamp = t_us;
    e.polarity = polarity;
    return e;
}

// landmark = x + (Ti - t) * flow, everything in microseconds (flow in px/us)
static bool testPropagate(){
    //three events at 0.7, 0.8, 0.9 seconds, at pixel locations (10,10), (11,10), (12,11)
    std::vector<Event> events = {
        makeEvent(10, 10, 700000),
        makeEvent(11, 10, 800000),
        makeEvent(12, 11, 900000)
    };

    Eigen::Vector2d flow(2e-6, 1e-6); //2 and 1 pixels per second
    Feature feature(Eigen::Vector2d(0.0, 0.0), 1e6 /*Ti = 1 s*/, 1.0, {}, events, flow, 15.0);
    feature.propagatePreviousEvents();

    const std::vector<Eigen::Vector2d> expected = {
        Eigen::Vector2d(10.6, 10.3), //10 + (1.0 - 0.7) * 2 = 10.6
        Eigen::Vector2d(11.4, 10.2), //11 + (1.0 - 0.8) * 2 = 11.4
        Eigen::Vector2d(12.2, 11.1)  //12 + (1.0 - 0.9) * 2 = 12.2
    };

    const auto& landmarks = feature.getLandmark();
    bool pass = landmarks.size() == expected.size();
    for (size_t i = 0; pass && i < landmarks.size(); ++i) {
        if ((landmarks[i] - expected[i]).norm() > 1e-9) pass = false;
    }
    std::cout << "propagate: " << (pass ? "passed" : "FAILED") << '\n';
    return pass;
}

// A synthetic L-shaped corner moving at a known flow: one EM window should recover that flow.
static bool testEMRecoversFlow(){
    const Eigen::Vector2d uTrue(4e-4, 2e-4); // px/us (400, 200 px/s)
    const double T0 = 1e6, dt = 6700.0;      // ~3 px of travel per window, like the adaptive window
    const double T1 = T0 + dt, T2 = T1 + dt;
    const Eigen::Vector2d corner(100.0, 100.0);

    std::vector<Eigen::Vector2d> pattern;
    for (int a = 0; a <= 6; a++) pattern.push_back(corner + Eigen::Vector2d(a, 0));
    for (int b = 1; b <= 6; b++) pattern.push_back(corner + Eigen::Vector2d(0, b));

    std::vector<Event> prev, buffer;
    for (double t = T0; t <= T2; t += 100.0) {
        for (const auto& p : pattern) {
            Eigen::Vector2d x = p + (t - T0) * uTrue;
            Event e = makeEvent((uint16_t)std::lround(x.x()), (uint16_t)std::lround(x.y()), (uint32_t)t);
            buffer.push_back(e);
            if (t < T1) prev.push_back(e);
        }
    }

    // feature sits on the corner at T1; prior = last window's flow, ~true in steady-state tracking.
    // (a wrong prior also smears the landmarks, so one window only partly corrects it; it settles over windows)
    Feature f(corner + (T1 - T0) * uTrue, T1, dt, {}, prev, uTrue, 15.0);
    int state = FeatureCollection(f, {}, buffer, 0.5, T2);

    const double err = (f.getFlow() - uTrue).norm() / uTrue.norm();
    const bool pass = state == HEALTHY && err < 0.1 && std::abs(f.getTi() - T2) < 1e-9;
    std::cout << "EM flow: (" << f.getFlow().x() * 1e6 << ", " << f.getFlow().y() * 1e6 << ") px/s, rel err "
              << err << " -> " << (pass ? "passed" : "FAILED") << '\n';
    return pass;
}

int main()
{
    bool pass = testPropagate();
    pass = testEMRecoversFlow() && pass;
    std::cout << (pass ? "Test passed!" : "Test failed!") << std::endl;
    return pass ? 0 : 1;
}