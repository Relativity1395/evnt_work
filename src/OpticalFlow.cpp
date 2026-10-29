#include <../include/OpticalFlow.hpp>
#include <../include/Feature.hpp>
#include <limits>
#include <cmath>
//#define MAXITR 50000
#define MAXITR 15


int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            F.assignAlive(INIT);
            return INIT;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window

        if (!F.isWindowComplete(highestTimestamp)) {
            return HEALTHY;
        }

        F.propagatePreviousEvents(); //turns previous events into landmarks

        if (F.getLandmark().empty()) {
            F.setConverged(false);

            if (F.checkHealth() > 3) {
                F.assignAlive(DEAD);
                return DEAD;
            }

            if (F.updateTimeWindow(highestTimestamp)) {
                F.setEvents();
            }

            return HEALTHY;
        }

        my_kd_tree_t mat_index(dim, F.getLandmark(), leaf);
        
        int iter = 0;
        double previousCost = std::numeric_limits<double>::infinity();
        double currentCost;
        bool converged = false;

        do {
            F.updateBackPropagation();
            F.generateKD(mat_index);

            if (!F.hasAssociations()) {
                break;
            }

            F.updateFlow();

            currentCost = F.calculateCost();

            if (std::abs(currentCost - previousCost) <= epsilon) {
                converged = true;
                break;
            }

            previousCost = currentCost;
            iter++;

        } while (iter < MAXITR);

        F.setConverged(converged);

        if (F.checkHealth() > 3){
            F.assignAlive(DEAD);
            return DEAD;
        }
        if (converged) {
            F.updatePosition(F.getDti());
            F.recordFlow();
        }
    }   
    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
    }
    

    return HEALTHY;

}
