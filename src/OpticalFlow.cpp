#include <../include/OpticalFlow.hpp>

void Feature::propagatePreviousEvents(double Ti ){
    landmark.clear();
    for(const Event& event : previousEvents){
        Eigen::Vector2d x = event.position;
        double t = event.timestamp;
        Eigen::Vector2d propagatedEvent = x + (Ti - t)*flow;
        landmark.push_back(propagatedEvent);
    }
} 

Feature::Feature(
    Eigen::Vector2d position,
    std::vector<Event> currentEvents,
    std::vector<Event> previousEvents,
    Eigen::Vector2d flow,
    double xi
){
    this->position = position;
    this -> currentEvents = currentEvents;
    this->previousEvents= previousEvents;
    this->flow = flow;
    this->xi = xi;
}

const std::vector<Eigen::Vector2d>& Feature::getLandmark() const{
    return landmark;
}
void Feature::setEvents(const std::vector<Event>& newEvents){
    this->previousEvents = this->currentEvents;
    this->currentEvents = newEvents;
}

void Feature::findEvents(const std::vector<Event>& E, double Ti, double dti){
    currentEvents.clear();
    for(const Event& event : E){
        Eigen::Vector2d x = event.position;
        double tbar = event.timestamp - Ti;
        Eigen::Vector2d u = flow;
        Eigen::Vector2d fti = position;
        if (event.timestamp >= Ti && event.timestamp <= (Ti + dti) ){
            Eigen::Vector2d V = (x - tbar * u) - fti;
            if(V.norm() <= xi){
                currentEvents.push_back(event);
            }
        }
    }
}