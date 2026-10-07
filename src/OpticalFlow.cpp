#include <../include/OpticalFlow.hpp>

#define MAXITR 50000
bool FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            return false;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window
        F.propagatePreviousEvents(); //turns previous events into landmarks
        F.calculateCost();
        int iter = 0;
        while (F.calculateCost() > epsilon && iter < MAXITR){
            F.generateKD(); //generates r_kj points
            F.updateFlow(); //updates optical flow
        }  
    }   
    F.updateTimeWindow(highestTimestamp); //updates timewindow
    F.setEvents();

    return true;

}
