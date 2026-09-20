#include <../include/OpticalFlow.hpp>

propagatePreviousEvent(const std::vector<Event>& Wi_1, const Eigen::Vector2d& Ui_1, double Ti ){
    std::vector<Eigen::Vector2d> propagatedEvents;
    for(Event& event : Wi_1){
        Eigen::Vector2d x = event.position;
        double t = event.timestamp;
        Eigen::Vector2d propagatedEvent = x + (Ti_1 - t)*Ui_1;
        propagatedEvents.push_back(propagatedEvent);
    }
    return propagatedEvents;
} 