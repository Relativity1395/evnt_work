#include "CalculateCost.hpp"

double calculateCost(
    const std::vector<Event>& currentEvents,
    const Eigen::Vector2d u,
    const std::vector<std::vector<Association>>& weights,
    const std::vector<Eigen::Vector2d>&landmarks,
    double Ti
){
    
        double cost = 0.0;

        for(std::size_t k = 0; k < currentEvents.size(); ++k){
            Eigen::Vector2d xk = currentEvents[k].position;
            double tbar = currentEvents[k].timestamp - Ti;
            for(const Association& association : weights[k]){
                double rkj = association.weight;
                std::size_t j = association.landmarkIndex;
                Eigen::Vector2d landmark = landmarks[j];
                Eigen::Vector2d mag = (xk - tbar * u) - landmark;
                cost += rkj * mag.squaredNorm();
                
            }
        }
            
        return cost;
        }
