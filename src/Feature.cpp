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
    bProp.clear();
    for(const Event& event : E){
        Eigen::Vector2d x = event.pos();
        double tbar = event.t() - Ti;
        Eigen::Vector2d u = flow;
        Eigen::Vector2d fti = position;
        Eigen::Vector2d backProp = x - tbar * u;
        Eigen::Vector2d V = backProp - fti;
        if (event.timestamp >= Ti && event.timestamp <= (Ti + dti)){
            if(V.norm() <= xi){
                currentEvents.push_back(event);
                bProp.push_back(backProp);
            }
        }
        }
    }


    bool Feature::findEventsInit(const std::vector<Event>& E){
    currentEvents.clear();   // otherwise repeated calls pile up duplicates
    bProp.clear();
    for (const Event& event : E){
        Eigen::Vector2d x = event.pos();
        double tbar = event.t() - Ti;
        Eigen::Vector2d backProp = x - tbar * flow;
        if ((backProp - position).norm() < xi){
            currentEvents.push_back(event);
            bProp.push_back(backProp);
            if (currentEvents.size() >= 300) break;
        }
    }
    return checkInitWindow();
}
// bool Feature::findEventsInit(const std::vector<Event>& E){
//     currentEvents.clear();
//     bProp.clear();

//     for(const Event& event : E){
//         Eigen::Vector2d x = event.pos();
//         double tbar = event.t() - Ti;
//         Eigen::Vector2d u = flow;
//         Eigen::Vector2d fti = position;
//         Eigen::Vector2d backProp = x - tbar * u;
//         Eigen::Vector2d V = backProp - fti;

//         if (E.size() <= 300 && V.norm() < xi){
//             currentEvents.push_back(event);
//             bProp.push_back(backProp);
//         }
//     } 
//     return checkInitWindow();  
// }
bool Feature::updateTimeWindow(double highestTimestamp){
    if (dti == 0){
        dti = currentEvents.back().t() - Ti;
        return true;
    }else{
        if (highestTimestamp >= Ti + dti){
            Ti = Ti + dti;
            dti = 3.0/getMedianMagnitude();
            if (dti >= MAXDELT){
                dti = MAXDELT;
            }
            return true;
        } 
    }
    return false;
}   
void Feature::propagatePreviousEvents(){
    landmark.clear();
    for(const Event& event : previousEvents){
        Eigen::Vector2d x = event.pos();
        double t = event.t();
        Eigen::Vector2d propagatedEvent = x + (Ti - t)*flow;
        landmark.push_back(propagatedEvent);
    }
}

void Feature::generateKD(){
    weights.clear();
    weights.resize(currentEvents.size());
    const double r2 = 4.2426;
    const int dim = 2;
    const int leaf = 10;

    weights.clear();
    weights.resize(bProp.size());      // one entry per current event, always aligned

    if (landmark.empty()) return;      // no landmarks yet: all weight lists stay empty

    typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
    my_kd_tree_t mat_index(dim, landmark, leaf);
    using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;

    for (size_t i = 0; i < bProp.size(); i++){
        Matches matches;
        mat_index.index->radiusSearch(bProp[i].data(), r2, matches);

        double sum = 0.0;
        for (const auto& m : matches){
            double score = std::exp(-m.second / 4.0);
            weights[i].push_back({m.first, score});
            sum += score;
        }
        if (sum > 0.0){
            for (auto& a : weights[i]) a.weight /= sum;
        }
    }
}
// void Feature::generateKD(){

//     const double r2 = 4.2426;
//     const int dim = 2;
//     const int leaf = 10;
    
//     typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
//     my_kd_tree_t mat_index(dim, landmark, leaf);
//     using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;
//     Matches ret_matches;
//     std::vector<Matches> totMatches;

//     for (size_t i = 0; i < bProp.size(); i++){
//         const size_t nMatches = mat_index.index->radiusSearch(bProp[i].data(), r2, ret_matches);
        
//         if (nMatches > 0){
//             totMatches.push_back(ret_matches);
//         }


//     }

//     for (std::size_t k = 0; k < totMatches.size(); ++k) {
//         auto& eventWeights = weights[k];
//         eventWeights.reserve(totMatches[k].size());

//         double sum = 0.0;

//         // Calculate gaussian scores and their sum
//         for (const auto& match : totMatches[k]) {
//             const double score = std::exp(-match.second / 4.0);

//             eventWeights.push_back({match.first, score});
//             sum += score;
//         }

//         // Divide each score by the sum to obtain rkj
//         for (auto& association : eventWeights) {
//             association.weight /= sum;
//         }
//     }



// }

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

int Feature::checkHealth(){
    badHealth = (!converged) && (currentEvents.size() < 5);
    if (badHealth){
        deadCycles++;
        
    }
    return deadCycles;
}

int Feature::getDeadCycles(){
    int dCycle = deadCycles;
    return dCycle;
}

double Feature::calculateCost(){
     double cost = 0.0;

    for(std::size_t k = 0; k < currentEvents.size(); ++k){
        Eigen::Vector2d xk = currentEvents[k].pos();
        double tbar = currentEvents[k].t() - Ti;
        for(const Association& association : weights[k]){
            double rkj = association.weight;
            std::size_t j = association.landmarkIndex;
            Eigen::Vector2d currLandmark = landmark[j];
            Eigen::Vector2d mag = (xk - tbar * flow) - currLandmark;
            cost += rkj * mag.squaredNorm();
            
        }
    }
            
        return cost;
}

Eigen::Vector2d Feature::updateFlow(){
    Eigen::Vector2d numerator(0.0,0.0);
        double denominator = 0.0;

        for(std::size_t k = 0; k < currentEvents.size(); ++k){
            Eigen::Vector2d xk = currentEvents[k].pos();
            double tbar = currentEvents[k].t() - Ti;
            for(const Association& association : weights[k]){
                double rkj = association.weight;
                std::size_t j = association.landmarkIndex;
                Eigen::Vector2d currLandmark = landmark[j];
                numerator += rkj*(xk - currLandmark) * tbar;
                denominator += rkj * std::pow(tbar, 2);
            }
        }
        Eigen::Vector2d u = numerator/denominator;
        return u;
}

bool Feature::checkWindow(){
    return (dti > 0 );
}

bool Feature::checkInitWindow(){
    return (currentEvents.size() >= 300);
}