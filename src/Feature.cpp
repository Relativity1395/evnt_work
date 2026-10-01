#include <../include/Feature.hpp>

Feature::Feature(
    Eigen::Vector2d position,
    std::vector<Event> currentEvents,
    std::vector<Event> previousEvents,
    Eigen::Vector2d flow,
    double xi
){
    this->position = position;
    this->currentEvents = currentEvents;
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
    bProp.clear();
    for(const Event& event : E){
        const double t = toSeconds(event.timestamp);
        if (t < Ti || t >(Ti + dti)){
            continue;
            //outside the window skip the math
        }
        const Eigen::Vector2d x = toVector2D(event.position);
        const double tbar = t - Ti;
        const Eigen::Vector2d backProp = x - tbar * flow;
        const Eigen::Vector2d V = backProp - position;
        
        if(V.norm() <= xi){
            currentEvents.push_back(event);
            bProp.push_back(backProp);
        }
    }
}

void Feature::propagatePreviousEvents(double Ti ){
    landmark.clear();
    for(const Event& event : previousEvents){
        const Eigen::Vector2d x = toVector2D(event.position);
        double t = event.timestamp;
        landmark.push_back(x + (toSeconds(t) - Ti) * flow);
    }
} 

std::vector<std::vector<nanoflann::ResultItem<size_t, double>>> Feature::generateKD(){

    const double r2 = 4.2426;
    const int dim = 2;
    const int leaf = 10;
    
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
    my_kd_tree_t mat_index(dim, landmark, leaf);
    using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;
    Matches ret_matches;
    std::vector<Matches> totMatches;
    for (size_t i = 0; i < bProp.size(); i++){
        const size_t nMatches = mat_index.index->radiusSearch(bProp[i].data(), r2, ret_matches);
        if (nMatches > 0){
            totMatches.push_back(ret_matches);
        }
    }

    return totMatches;

}