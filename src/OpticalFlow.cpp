#include <../include/OpticalFlow.hpp>
void initFeature(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    F.findEvents(currEvents);
        F.propagatePreviousEvents();
        
        if (F.checkWindow()){
            while (F.calculateCost() > epsilon){
                F.generateKD();
                F.updateFlow();
            }
            
        }
        if (F.checkInitWindow()){
            F.updateTimeWindow(highestTimestamp);
            F.setEvents();
        }
}
