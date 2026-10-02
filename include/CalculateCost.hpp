#include <vector>
#include <Eigen/Dense>
#include "Types.hpp"
#include "WeightDist.hpp"


double calculateCost(
    const std::vector<Event>& currentEvents,
    const Eigen::Vector2d u,
    const std::vector<std::vector<Association>>& weights,
    const std::vector<Eigen::Vector2d>&landmarks,
    double Ti
);
