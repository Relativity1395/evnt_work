#include <../include/OpticalFlow.hpp>
#include <iostream>

int main(){

    Event event1;
    event1.position = Eigen::Vector2d(0,0);
    event1.timestamp = 1.0;
    Eigen::Vector2d Ui = Eigen::Vector2d(2,1);
    double Ti = 4.0;
    Event event2;
    event2.position = Eigen::Vector2d(10,5);
    event2.timestamp = 2.0;
    Event event3;
    event3.position = Eigen::Vector2d(20,10);
    event3.timestamp = 3.0;
    std::vector<Event> Events;
    Events.push_back(event1);
    Events.push_back(event2);
    Events.push_back(event3);
    std::vector<Eigen::Vector2d> result = propagatePreviousEvent(Events, Ui, Ti );
    std::cout << result[0].x() << "," << result[0].y() << std::endl; // 6,3
    std::cout << result[1].x() << "," << result[1].y() << std::endl; // 14,7
    std::cout << result[2].x() << "," << result[2].y() << std::endl; // 22,11

}

