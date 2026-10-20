#include <../include/OpticalFlow.hpp>
#include <cmath>
#include <limits>



int FeatureCollection(Feature& F, const std::vector<Event>& newEvents, const std::vector<Event>& eventBuffer, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(newEvents)){
            return INIT;
        }
        F.updateTimeWindow(highestTimestamp);
        F.setEvents();
        return HEALTHY;
    }

    if (!F.windowClosed(highestTimestamp)) return HEALTHY; //[Ti, TI+dti] not full yet, nothing ever happens
    
    F.findEvents(eventBuffer); //colects current evnts wiithin spatiotemperol window
    F.propagatePreviousEvents();

    //change in cost between iterations; styas infinite if EM cant run
    double delta = std::numeric_limits<double>::infinity();
    if (!F.getLandmark().empty() && F.eventCount() > 0) {
        LandmarkTree tree(2, F.getLandmark(), 10); //landmark are fixed for the whole window: build once
        F.generateKD(tree);// generate r_kj with the prior flow
        double cost = F.calculateCost();

        for (int iter = 0; iter <MAXITR && delta > epsilon; iter++){
            F.updateFlow(); //M step
            F.generateKD(tree); // E step with the new flow
            double newCost = F.calculateCost();
            delta = std::abs(cost - newCost);
            cost = newCost;
        }
    }

    int state = HEALTHY;
    if (F.checkHealth(delta,epsilon) > MAXHEALTH) {
        F.killFeature();
        state = DEAD;
    }

    if (F.updateTimeWindow(highestTimestamp)) {
        F.setEvents();
    }

    return state;
}


