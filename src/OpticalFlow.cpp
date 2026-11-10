#include <../include/OpticalFlow.hpp>
#include <../include/Feature.hpp>


constexpr double FLOW_TOL_PX = 0.05;  
// constexpr double COST_TOL    = epsilo;   

int FeatureCollection(Feature& F, const EventGrid& grid, const std::vector<Event>& currEvents, const double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(grid, currEvents)){
            if (highestTimestamp - F.getTi() > MAX_INT_TIME) {
                F.markDead();
                F.assignAlive(DEAD);
                return DEAD;
            }
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
            int iter = F.setIters(true);
            do {
                previousFlow = F.getFlow();
                F.updateBackPropagation();
                F.generateKD(mat_index);
                newFlow = F.updateFlow();
                iter = F.setIters(false);

                
            } while (F.calculateCost() > epsilon && iter < MAXITR);
                
            
            // F.setIters(iter);
            F.setConverged(F.calculateCost() <= epsilon);

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
    F.assignAlive(HEALTHY);
    return HEALTHY;
}