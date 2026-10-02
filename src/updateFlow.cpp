#include "updateFlow.hpp"
#include <cmath>



Eigen::Vector2d updateFlow(
        const   std::vector<Event>& currentEvents,
        const std::vector<Eigen::Vector2d>& landmarks,
        const std::vector<std::vector<Association>>& weights,
        double Ti){
            Eigen::Vector2d numerator(0.0,0.0);
            double denominator = 0.0;

            for(std::size_t k = 0; k < currentEvents.size(); ++k){
                Eigen::Vector2d xk = currentEvents[k].position;
                double tbar = currentEvents[k].timestamp - Ti;
                for(const Association& association : weights[k]){
                    double rkj = association.weight;
                    std::size_t j = association.landmarkIndex;
                    Eigen::Vector2d landmark = landmarks[j];
                    numerator += rkj*(xk - landmark) * tbar;
                    denominator += rkj * std::pow(tbar, 2);
                }
            }
            Eigen::Vector2d u = numerator/denominator;
            return u;
        }