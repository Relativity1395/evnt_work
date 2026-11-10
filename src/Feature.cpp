#include <../include/Feature.hpp>

Feature::Feature(
    Eigen::Vector2d position,
    double Ti,
    double dti,
    std::vector<Event> currentEvents,
    std::vector<Event> previousEvents,
    Eigen::Vector2d flow,
    double xi)
{
    this->position = position;
    this->currentEvents = currentEvents;
    this->previousEvents= previousEvents;
    this->flow = flow;
    this->Ti = Ti;
    this->dti = dti;
    this->xi = xi;
}
void Feature::updatePosition(double deltaTime) {
    position += flow * deltaTime;
}
const std::vector<Eigen::Vector2d>& Feature::getLandmark() const{
    return landmark;
}
double Feature::getDti() const {
    return dti;
}
Eigen::Vector2d Feature::getFlow() const{
    return flow;
}
bool Feature::isWindowComplete(double highestTimestamp) const {
    return dti > 0 && highestTimestamp >= Ti + dti;
}
void Feature::setConverged(bool status){
    converged = status;
}
size_t Feature::getEventCount() const {
    return currentEvents.size();
}
bool Feature::hasConverged() const {
    return converged;
}
void Feature::setEvents(){
    //swap keeps both buffer capacity, so the next window doesnt reallocate
    std::swap(previousEvents, currentEvents);
    currentEvents.clear();
    bProp.clear();
}
void Feature::findEvents(const EventGrid& grid, const std::vector<Event>& E){
    Eigen::Vector2d end = position + dti * flow;
    double x0 = std::min(position.x(), end.x()) - xi, x1 = std::max(position.x(), end.x()) + xi;
    double y0 = std::min(position.y(), end.y()) - xi, y1 = std::max(position.y(), end.y()) + xi;

    grid.forEachInBox(x0, y0, x1, y1, [&](int idx){
        const Event& event = E[idx];
        double t = event.t();
        if (t < Ti || t >= Ti + dti) return;
        Eigen::Vector2d backProp = event.pos() - (t - Ti) * flow;
        if ((backProp - position).norm() <= xi) currentEvents.push_back(event);
    });
}



    bool Feature::findEventsInit(const EventGrid& grid, const std::vector<Event>& E){
    grid.forEachInBox(position.x() - xi, position.y() - xi, position.x() + xi, position.y() + xi, [&](int idx){
        if (currentEvents.size() >= MIN_EVNT_INIT) return;
        const Event& event = E[idx];
        if (event.t() < Ti) return;
        Eigen::Vector2d backProp = event.pos() - (event.t() - Ti) * flow;
        if ((backProp - position).norm() < xi) currentEvents.push_back(event);
    });
    return checkInitWindow();
}

bool Feature::updateTimeWindow(double highestTimestamp){

    // Initial time window
    if (dti == 0){

        if (currentEvents.empty()){
            return false;
        }

        double tMax = std::max_element(currentEvents.begin(), currentEvents.end(),
        [](const Event& a, const Event& b){ return a.t() < b.t(); })->t();
        dti = tMax - Ti;

        if (dti <= 0){
            return false;
        }

        if (dti > MAX_INT_TIME){
            dti = MAX_INT_TIME;
        }
        Ti += dti;
        return true;
    }

    // Check whether the current window is complete
    if (highestTimestamp >= Ti + dti){

        double median = getMedianMagnitude();

        double newDti;

        if (median > 1e-12){
            newDti = 3.0 / median;
        }else{
            newDti = MAXDELT;
        }

        Ti = Ti + dti;

        dti = std::min(newDti, static_cast<double>(MAXDELT));

        return true;
    }

    return false;
}

void Feature::subsampleEvents(size_t maxN){
    if (currentEvents.size() <= maxN) return;
    double step = (double)currentEvents.size() / maxN;
    std::vector<Event> kept;
    kept.reserve(maxN);
    for (size_t k = 0; k < maxN; k++) kept.push_back(currentEvents[(size_t)(k * step)]);
    currentEvents.swap(kept);
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


void Feature::updateBackPropagation(){
    bProp.clear();

    for (const Event& event : currentEvents){
        double tbar = (event.t() - Ti);

        Eigen::Vector2d backProp =
            event.pos() - tbar * flow;

        bProp.push_back(backProp);
    }
}
void Feature::generateKD(my_kd_tree_t& tree){
    weights.resize(bProp.size());
    for (auto& w : weights) w.clear();         
    if (landmark.empty()) return;

    Matches matches;
    nanoflann::SearchParameters params;
    params.sorted = false;

    for (size_t i = 0; i < bProp.size(); i++){
        tree.index->radiusSearch(bProp[i].data(), r2, matches, params);
        double sum = 0.0;
        for (const auto& m : matches){
            double score = std::exp(-m.second / 4.0);
            weights[i].push_back({m.first, score});
            sum += score;
        }
        if (sum > 0.0) for (auto& a : weights[i]) a.weight /= sum;
    }
}


Eigen::Vector2d Feature::getPosition() const {
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
        magnitudes.push_back(vec);
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
    badHealth = (!converged) || (currentEvents.size() < MIN_EVNT_DEATH);
    // std::cout << "Badhealth: " << badHealth <<" Current events: " << currentEvents.size() << std::endl;
    if (badHealth == true){
        deadCycles++;
        
    }
    else{
        deadCycles = 0;
    }
    return deadCycles;
}

int Feature::getDeadCycles() const {
    int dCycle = deadCycles;
    return dCycle;
}

double Feature::calculateCost() {
    cost = 0.0;
    size_t matched = 0;
    for (size_t k = 0; k < currentEvents.size(); ++k){
        if (weights[k].empty()) continue;
        matched++;
        Eigen::Vector2d bp = currentEvents[k].pos() - (currentEvents[k].t() - Ti) * flow;
        for (const Association& a : weights[k])
            cost += a.weight * (bp - landmark[a.landmarkIndex]).squaredNorm();
    }
    cost = matched ? cost / matched : std::numeric_limits<double>::infinity();
    return cost;
}

Eigen::Vector2d Feature::updateFlow(){
    Eigen::Vector2d numerator(0.0,0.0);
        double denominator = 0.0;

        for(std::size_t k = 0; k < currentEvents.size(); ++k){
            Eigen::Vector2d xk = currentEvents[k].pos();
            double tbar = (currentEvents[k].t() - Ti);

            // std::cout << tbar << std::endl;
            // std::cout << Ti << std::endl;
            // std::cout << currentEvents[k].t() << std::endl;
            for(const Association& association : weights[k]){
                double rkj = association.weight;
                std::size_t j = association.landmarkIndex;
                Eigen::Vector2d currLandmark = landmark[j];
                numerator += rkj*(xk - currLandmark) * tbar;
                denominator += rkj * tbar*tbar;
            }
        }
        
        if (denominator <= 1e-12) {
            return flow;
        }       
        Eigen::Vector2d u = numerator/denominator;
        flow = u;
        t_flow.push_back(flow.norm());
        if ((int)t_flow.size() > 5){
            t_flow.erase(t_flow.begin());
        }
        return u;
}

bool Feature::checkWindow(){
    return (dti > 0 );
}

bool Feature::checkInitWindow(){
    return (currentEvents.size() >= MIN_EVNT_INIT);
}