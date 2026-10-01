#include "../include/Feature.hpp"
#include <iostream>


//Build an event the way the driver does : integer pixels, microsecond timestamp.
static Event makeEvent(uint16_t x, uint16_t y, uint32_t t_us, bool polarity =true){
    Event e;
    e.position << x, y;
    e.timestamp = t_us;
    e.polarity = polarity;
    return e;
}
int main()
{
    //three events at 0.7, 0.8, 0.9 seconds, at pixel locations (10,10), (11,10), (12,11)
    std::vector<Event> events = {
        makeEvent(10, 10, 700000),
        makeEvent(11, 10, 800000),
        makeEvent(12, 11, 900000)
    };
    

    Eigen::Vector2d position(0.0, 0.0);
    Eigen::Vector2d flow(2.0, 1.0); //pixels per second

    Feature feature(
        position,
        events,
        events,
        flow,
        0.0
    );

    feature.propagatePreviousEvents(1.0); //Ti = 1.0 seconds

    //landmark = x + (Ti - t) * flow
    const std::vector<Eigen::Vector2d> expected = {
        Eigen::Vector2d(10.6, 10.3), //10 + (1.0 - 0.7) * 2 = 10 + 0.3 * 2 = 10 + 0.6 = 10.6
        Eigen::Vector2d(11.4, 10.2), //11 + (1.0 - 0.8) * 2 = 11 + 0.2 * 2 = 11 + 0.4 = 11.4
        Eigen::Vector2d(12.2, 11.1) //12 + (1.0 - 0.9) * 2 = 12 + 0.1 * 2 = 12 + 0.2 = 12.2
    };

    const auto& landmarks = feature.getLandmark();

    bool pass = landmarks.size() == expected.size();
    for (size_t i = 0; i < landmarks.size(); ++i) {
        std::cout <<landmarks[i].x() << "," << landmarks[i].y() << '\n';
        if (i < expected.size() && (landmarks[i] - expected[i]).norm() > 1e-9) {
            pass = false;
        }
    }

    std::cout << (pass ? "Test passed!" : "Test failed!") << std::endl;
    return pass ? 0 : 1;
}