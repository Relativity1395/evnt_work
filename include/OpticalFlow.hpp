#include "Feature.hpp"
#include "CalculateCost.hpp"
#include "WeightDist.hpp"
#include "updateFlow.hpp"
void checkInitFeature(Eigen::Vector2d);

Eigen::Vector2d em1(const Feature& feature,
                    const std::vector<Event>& currentEvents,
                    const std::vector<Eigen::Vector2d>& landmarks,
                    Eigen::Vector2d ui,
                    double Ti,
                    double epsilon1,
                    int maxIterations
                );
