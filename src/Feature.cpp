#include <../include/Feature.hpp>
#include <algorithm>
#include <cmath>

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
    this->currentEvents = std::move(currentEvents);
    this->previousEvents= std::move(previousEvents);
    this->flow = flow;
    this->Ti = Ti;
    this->dti = dti;
    this->xi = xi;
}

const std::vector<Eigen::Vector2d>& Feature::getLandmark() const{
    return landmark;
}

void Feature::setEvents(){
    //swap keeps both buffer capacity, so the next window doesnt reallocate
    std::swap(previousEvents, currentEvents);
    currentEvents.clear();
}
//E must be sorted by timestamp, so we get right into Ti
void Feature::findEvents(const std::vector<Event>& E){
    currentEvents.clear();
   
    const double tEnd = Ti + dti;
    const double xi2 = xi * xi;
    
    auto it = std::lower_bound(E.begin(), E.end(), Ti,
    [](const Event& event, double t) {
        return event.t() < t;});
    
    for (; it != E.end() && it->t() <= tEnd; ++it) {
        const Eigen::Vector2d backProp = it->pos() - (it->t() - Ti) * flow;
        if ((backProp - position).squaredNorm() < xi2) {
            currentEvents.push_back(*it);
        }
    }
}

bool Feature::findEventsInit(const std::vector<Event>& E){
    const double xi2 = xi * xi;

    for (const Event& event : E) {
        if (currentEvents.size() >= MINEVNT) break; // Stop if we have enough events
        if (event.t() < Ti) continue; // Skip events before Ti
        const Eigen::Vector2d backProp = event.pos() - (event.t() - Ti) * flow;
        if ((backProp - position).squaredNorm() < xi2) {
            currentEvents.push_back(event);
        }
    }
    return checkInitWindow();
}


bool Feature::updateTimeWindow(double highestTimestamp){
    if (dti == 0){
        //init fiest window length = span of the init event, it starts where init ended
        dti = std::max(currentEvents.back().t() - Ti, 1.0);
        Ti += dti;
        return true;
    }else{
        if (highestTimestamp >= Ti + dti){
            position += dti * flow;
            Ti += dti;
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
    landmark.reserve(previousEvents.size());
    for (const Event& event : previousEvents) {
        landmark.push_back(event.pos() + (Ti - event.t()) * flow);
    }
}

// E step : back prop current events with the current flow and weight them against the landmakrrs
void Feature::generateKD(const LandmarkTree& tree){

    const double r2 = 4.2426;
    nanoflann::SearchParameters params;
    params.sorted = false; // We don't need sorted results for our use case
    weights.resize(currentEvents.size());

    for (std::size_t k = 0; k < currentEvents.size(); ++k) {
        auto& eventWeights = weights[k];
        eventWeights.clear();

        const Eigen::Vector2d backProp = currentEvents[k].pos() - (currentEvents[k].t() - Ti) * flow;
        if (tree.index->radiusSearch(backProp.data(), r2, matches, params) == 0)  continue;

        double sum = 0.0;
        //calculare gaussian scores and their sum
        for (const auto& match : matches) {
            const double score = std::exp(-match.second / 4.0);
            eventWeights.push_back({match.first, score});
            sum += score;
        }
        // divide each score by the sum to get rkj
        for (auto& association : eventWeights) {
            association.weight /= sum;
        }
    }

}

const Eigen::Vector2d& Feature::getPosition() const{
    return position;
}

const Eigen::Vector2d& Feature::getFlow() const{
    return flow;
}

double Feature::getTi() const{
    return Ti;
}

std::size_t Feature::eventCount() const{
    return currentEvents.size();
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



int Feature::checkHealth(double cost, double epsilon){
    converged = cost < epsilon;
    badHealth = (!converged) || (currentEvents.size() < MINEVNT);
    deadCycles = badHealth ? deadCycles + 1 : 0;
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
            Eigen::Vector2d mag = (xk - tbar *flow) - landmark[association.landmarkIndex];
            cost += association.weight * mag.squaredNorm();
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
                numerator += rkj*(xk - landmark[association.landmarkIndex]) * tbar;
                denominator += rkj * tbar * tbar;
            }
        }
        if (denominator > 0.0) {
            flow = numerator / denominator; //no matches: keep the previous flow
        }
    return flow;
}

bool Feature::checkWindow() const{
    return (dti > 0);
}

bool Feature::windowClosed(double highestTimestamp) const{
    return (highestTimestamp >= Ti + dti);
}

bool Feature::checkInitWindow(){
    return (currentEvents.size() >= 300);
}

void Feature::killFeature(){
    dead = true;
}
bool Feature::getLife() const{
    return dead;
}