#include <../include/OpticalFlow.hpp>

//#define MAXITR 50000
#define MAXITR 20

int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            return INIT;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window
        if (!F.isWindowComplete(highestTimestamp)){
            return HEALTHY;
        }
        F.propagatePreviousEvents(); //turns previous events into landmarks

        int iter = INIT;
        Eigen::Vector2d previousFlow;
        Eigen::Vector2d newFlow;
        do {
        previousFlow = F.getFlow();

        F.updateBackPropagation();
        F.generateKD();

        newFlow = F.updateFlow();

        iter++;

    }   while ((newFlow - previousFlow).norm() > epsilon && iter < MAXITR);
        F.setConverged(
        (newFlow - previousFlow).norm() <= epsilon
        );
        if (F.checkHealth() > 3){
            return DEAD;
        }
        F.updatePosition(F.getDti());
    }   
    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
    }
    

    return HEALTHY;

}
