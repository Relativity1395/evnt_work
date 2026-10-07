#include "Feature.hpp"
#include "../include/Types.hpp"
#include <stdlib.h>
void checkInitFeature(Event& e);
bool FeatureCollection(Feature& F, const std::vector<Event>& currEvents, double epsilon, double highestTimestamp);