#include <../include/OpticalFlow.hpp>

#define MAXITR 50000
int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            return INIT;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window
        F.propagatePreviousEvents(); //turns previous events into landmarks
        F.calculateCost();
        int iter = INIT;
        while (F.calculateCost() > epsilon && iter < MAXITR){
            if (iter != 0){
                F.generateKD(); //generates r_kj points
            }
            F.updateFlow(); //updates optical flow
            iter++;
        }  
        if (F.checkHealth() > 3){
            return DEAD;
        }

    }   
    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
    }
    

    return HEALTHY;

}
