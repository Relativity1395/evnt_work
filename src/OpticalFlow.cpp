#include <../include/OpticalFlow.hpp>
#include <limits>
#include <../include/Feature.hpp>

Eigen::Vector2d em1(Feature& feature,
                    const std::vector<Event>& currentEvents,
                    Eigen::Vector2d ui,
                    double Ti,
                    double epsilon1,
                    int maxIterations
                ){
    
    Eigen::Vector2d u = ui;
    double cost = std::numeric_limits<double>::infinity();
    int iteration = 0;

    while( cost > epsilon1 && iteration < maxIterations){
        auto matches = feature.generateKD();
        auto weights = weightDist(matches);
        u = updateFlow(currentEvents, feature.getLandmark(), weights, Ti);

        cost = calculateCost(currentEvents, u, weights, feature.getLandmark(), Ti);
        iteration++;
    }
    return u;
}
