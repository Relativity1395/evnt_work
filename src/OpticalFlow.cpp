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

Feature::Feature(
    Eigen::Vector2d position,
    std::vector<Event> events,
    Eigen::Vector2d flow
){
    this->position = position;
    this -> events = events;
    this->flow = flow;
}

const std::vector<Eigen::Vector2d>& Feature::getLandmark() const{
    return landmark;
}
void Feature::setEvents(const std::vector<Event>& newEvents){
    this->events = newEvents;
}