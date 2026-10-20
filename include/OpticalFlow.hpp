#include "Feature.hpp"
#include "../include/Types.hpp"
#include <stdlib.h>
#define MAXITR 100 // EM normaly converges in < 20 
#define MAXHEALTH 3
enum FeatureState {
    INIT = 0,
    DEAD = 1, 
    HEALTHY = 2
};
void checkInitFeature(Event& e);
int FeatureCollection(Feature& F, const std::vector<Event>& newEvents, const std::vector<Event>& eventBuffer, 
    double epslion, double highestTimestamp);