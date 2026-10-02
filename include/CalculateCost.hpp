#include <vector>
#include <Eigen/Dense>
#include "Types.hpp"
#include "WeightDist.hpp"


double calculateCost(
    std::vector<Event>& currentEvents,
    Eigen::Vector2d u,
    std::vector<std::vector<Association>>& weights,
    std::vector<Eigen::Vector2d>&landmarks,
    double Ti
);
