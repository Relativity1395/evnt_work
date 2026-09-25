#include "../include/Feature.hpp"
#include <iostream>

int main()
{
    Event event1;
    event1.position = Eigen::Vector2d(10.0, 10.0);
    event1.timestamp = 0.7;

    Event event2;
    event2.position = Eigen::Vector2d(10.2, 10.1);
    event2.timestamp = 0.8;

    Event event3;
    event3.position = Eigen::Vector2d(10.4, 10.2);
    event3.timestamp = 0.9;

    std::vector<Event> events = {event1, event2, event3};

    Eigen::Vector2d position(0.0, 0.0);
    Eigen::Vector2d flow(2.0, 1.0);

    Feature feature(
        position,
        events,
        events,
        flow,
        {},
        0.0
    );

    feature.propagatePreviousEvents(1.0);

    const auto& landmarks = feature.getLandmark();

    for (const auto& landmark : landmarks)
        std::cout << landmark.x() << ", " << landmark.y() << '\n';

    return 0;
}