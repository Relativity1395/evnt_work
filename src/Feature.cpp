#include <../include/Feature.hpp>

Feature::Feature(
    Eigen::Vector2d position,
    double Ti,
    double dti,
    std::vector<Event> currentEvents,
    std::vector<Event> previousEvents,
    Eigen::Vector2d flow,
    double xi
){
    this->position = position;
    this->currentEvents = currentEvents;
    this->previousEvents= previousEvents;
    this->flow = flow;
    this->Ti = Ti;
    this->dti = dti;
    this->xi = xi;
}

const std::vector<Eigen::Vector2d>& Feature::getLandmark() const{
    return landmark;
}

void Feature::setEvents(){
    this->previousEvents = this->currentEvents;
    currentEvents.clear();
    // this->currentEvents = newEvents;
}

void Feature::findEvents(const std::vector<Event>& E){
    currentEvents.clear();
    for(const Event& event : E){
        Eigen::Vector2d x = event.position;
        double tbar = event.timestamp - Ti;
        Eigen::Vector2d u = flow;
        Eigen::Vector2d fti = position;
        Eigen::Vector2d backProp = x - tbar * u;
        Eigen::Vector2d V = backProp - fti;
        if (dti == 0){
            if(previousEvents.size() < 300 && x.norm() <= xi){
                currentEvents.insert(currentEvents.end(), previousEvents.begin(), previousEvents.begin());
                currentEvents.push_back(event);
                bProp.push_back(backProp);
            }
        }else{
            if (event.timestamp >= Ti && event.timestamp <= (Ti + dti)){
                if(V.norm() <= xi){
                    currentEvents.push_back(event);
                    bProp.push_back(backProp);
                }
            }
        }
    }
}   

void Feature::updateTimeWindow(){
    if (dti == 0){
        if (currentEvents.size() < 300){
            dti = 0;
        }else if (currentEvents.size() >= 300){
            dti = currentEvents.back().timestamp - Ti;
        }
    }else{
        Ti = Ti + dti;
        dti = 3.0/getMedianMagnitude();
        if (dti >= 1000000){
            dti = 1000000;
        }
    }
}
void Feature::propagatePreviousEvents(){
    landmark.clear();

    if (dti != 0){
        for(const Event& event : previousEvents){
            Eigen::Vector2d x = event.position;
            double t = event.timestamp;
            Eigen::Vector2d propagatedEvent = x + (Ti - t)*flow;
            landmark.push_back(propagatedEvent);
        }
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

Eigen::Vector2d Feature::getPosition(){
    Eigen::Vector2d pos = position;
    return pos;
}

void updateFeature(){

}
double Feature::getMedianMagnitude() {
    if (t_flow.empty()) return 0.0; // Handle empty edge case safely

    // 1. Extract the magnitude (norm) of each Eigen vector
    std::vector<double> magnitudes;
    magnitudes.reserve(t_flow.size()); // Pre-allocate memory for performance
    
    for (const auto& vec : t_flow) {
        magnitudes.push_back(vec.norm());
    }

    // 2. Perform a partial linear-time O(N) sort for the median
    size_t n = magnitudes.size() / 2;
    std::nth_element(magnitudes.begin(), magnitudes.begin() + n, magnitudes.end());

    // 3. Return the median based on odd/even counts
    if (magnitudes.size() % 2 != 0) {
        // Odd number of elements: return the exact middle element
        return magnitudes[n];
    } else {
        // Even number of elements: average the two middle values
        // std::max_element finds the largest value in the unsorted left half
        auto max_it = std::max_element(magnitudes.begin(), magnitudes.begin() + n);
        return (*max_it + magnitudes[n]) / 2.0;
    }
}