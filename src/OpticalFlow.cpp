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
    std::vector<Eigen::Vector2d> bProp,
    double xi
){
    this->position = position;
    this -> currentEvents = currentEvents;
    this->previousEvents= previousEvents;
    this->flow = flow;
    this->bProp = bProp;
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
        Eigen::Vector2d backProp = x - tbar * u;
        if (event.timestamp >= Ti && event.timestamp <= (Ti + dti) ){
            Eigen::Vector2d V = backProp - fti;
            
            if(V.norm() <= xi){
                currentEvents.push_back(event);
                bProp.push_back(backProp);
            }
        }
    }

    
}

void Feature::generateKD(){

    const double r2 = 4.2426;
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
    my_kd_tree_t mat_index(2, landmark, 10);

    using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;
    Matches ret_matches;
    for (int i = 0; i < bProp.size(); i++){
    const size_t nMatches = mat_index.index->radiusSearch(bProp[i].data(), r2, ret_matches);
    }


}
