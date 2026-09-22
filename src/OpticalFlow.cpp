#include <../include/OpticalFlow.hpp>

void Feature::propagatePreviousEvents(double Ti){
    landmark_.clear();
    for(const Event& event : events_){
        Eigen::Vector2d x = event.position;
        double t = event.timestamp;
        Eigen::Vector2d propagatedEvent = x + (Ti - t)*flow_;
        landmark_.push_back(propagatedEvent);
    }
} 