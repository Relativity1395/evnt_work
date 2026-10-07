#include "Feature.hpp"
#include "../include/Types.hpp"
#include <stdlib.h>

enum FeatureState {
    INIT = 0,
    DEAD = 1, 
    HEALTHY = 2
};
void checkInitFeature(Event& e);
int FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp);