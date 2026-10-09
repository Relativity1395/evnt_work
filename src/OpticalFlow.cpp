#include <../include/OpticalFlow.hpp>


int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            return INIT;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window
        F.propagatePreviousEvents(); //turns previous events into landmarks
        F.generateKD();
        double cost = F.calculateCost();

        int iter = INIT;
        while (cost > epsilon && iter < MAXITR){
            if (iter != 0){
                F.generateKD(); //generates r_kj points
            }
            F.updateFlow(); //updates optical flow
            iter++;
        }  
        if (F.checkHealth(cost, epsilon) > MAXHEALTH){
            F.killFeature();
        }

    }   
    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
    }
    

    return HEALTHY;

}
