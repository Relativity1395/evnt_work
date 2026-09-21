#include <../include/OpticalFlow.hpp>

void Feature::propagatePreviousEvents(double Ti ){
    landmark.clear();
    for(const Event& event : events){
        Eigen::Vector2d x = event.position;
        double t = event.timestamp;
        Eigen::Vector2d propagatedEvent = x + (Ti - t)*flow;
        landmark.push_back(propagatedEvent);
    }
} 