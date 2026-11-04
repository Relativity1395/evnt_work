#include <../include/OpticalFlow.hpp>
#include <../include/Feature.hpp>
#define MAXITR 50

constexpr double FLOW_TOL_PX = 0.05;  
constexpr double COST_TOL    = 2.0;   

int FeatureCollection(Feature& F, const EventGrid& grid, const std::vector<Event>& currEvents, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(grid, currEvents)){
            F.assignAlive(INIT);
            return INIT;
        }
    } else {
        F.findEvents(grid, currEvents);
        if (!F.isWindowComplete(highestTimestamp)) return HEALTHY;

        F.subsampleEvents(MAX_WINDOW_EVENTS);
        F.propagatePreviousEvents();

        if (!F.getLandmark().empty()) {
            my_kd_tree_t mat_index(dim, F.getLandmark(), leaf);
            Eigen::Vector2d previousFlow, newFlow;
            int iter = 0;
            do {
                previousFlow = F.getFlow();
                F.updateBackPropagation();
                F.generateKD(mat_index);
                newFlow = F.updateFlow();
                iter++;
            } while ((newFlow - previousFlow).norm() * F.getDti() > FLOW_TOL_PX && iter < MAXITR);

            // F.setIters(iter);
            F.setConverged(F.calculateCost() <= COST_TOL);

            if (F.checkHealth() > 3){
                F.assignAlive(DEAD);
                return DEAD;
            }
            F.updatePosition(F.getDti());
        }
    }

    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
        F.findEvents(grid, currEvents);  
    }
    return HEALTHY;
}