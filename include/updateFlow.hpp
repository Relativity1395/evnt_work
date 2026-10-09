#pragma once
#include <vector>
#include <Eigen/Dense>
#include "Types.hpp"
#include "WeightDist.hpp"

Eigen::Vector2d updateFlow(
        const   std::vector<Event>& currentEvents,
        const std::vector<Eigen::Vector2d>& landmarks,
        const std::vector<std::vector<Association>>& weights,
        double Ti);