#include <../include/OpticalFlow.hpp>
#include <../include/Feature.hpp>
#include <cmath>
#include <limits>
#define MAXITR 20 // relative-change stop usually exits in a few iterations; this is only a cap




int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(currEvents)){
            F.assignAlive(INIT);
            return INIT;
        }
    }else{
        F.findEvents(currEvents); //collects current events within spatiotmeporal window
        if (!F.isWindowComplete(highestTimestamp)){
            return HEALTHY;
        }
        F.propagatePreviousEvents(); //turns previous events into landmarks
        if (F.getLandmark().empty()) {
            return HEALTHY;   // nothing to match against this window; skip EM
        }
        my_kd_tree_t mat_index(dim, F.getLandmark(), leaf); // landmarks are fixed for the whole window, build once

        // EM: stop when the cost stops improving (relative change <= epsilon), not on an absolute
        // cost, because the cost is a SUM over matches and grows with the number of events.
        int iter = 0;
        double cost = 0.0;
        double prevCost = std::numeric_limits<double>::max();
        bool converged = false;
        while (iter < MAXITR) {
            F.updateBackPropagation();
            F.generateKD(mat_index);
            F.updateFlow();
            cost = F.calculateCost();
            iter++;
            if (std::abs(prevCost - cost) <= epsilon * prevCost) { converged = true; break; }
            prevCost = cost;
        }
        F.setIters(iter);
        // shortcut: cost == 0 means no event matched a landmark, so "no change" is not convergence;
        // upgrade to a matched-event count if features with few matches still slip through.
        F.setConverged(converged && cost > 0.0);
        if (F.checkHealth() > 3){
            F.assignAlive(DEAD);
            return DEAD;
        }
        F.updatePosition(F.getDti());
    }   
    if (F.updateTimeWindow(highestTimestamp)){
        F.setEvents();
        // the window just advanced: collect this packet's events that fall in the NEW window,
        // otherwise they are never seen and the next window starts (nearly) empty
        F.findEvents(currEvents);
    }
    

    return HEALTHY;

}
