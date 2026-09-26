#pragma once

#include <cstddef>
#include <vector>
#include "nanoflann.hpp"

struct Association { //used for landmarks index and its calculated weight
    std::size_t landmarkIndex;
    double weight;
};

using LandmarkMatches = //nearby landmarks and their squared distances for one event
    std::vector<nanoflann::ResultItem<std::size_t, double>>;

std::vector<std::vector<Association>> weightdist(
    const std::vector<LandmarkMatches>& matches
);