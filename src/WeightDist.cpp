#include "WeightDist.hpp"

#include <cmath>

std::vector<std::vector<Association>> weightdist(
    const std::vector<LandmarkMatches>& matches
)
{
    std::vector<std::vector<Association>> weights(matches.size());

    for (std::size_t k = 0; k < matches.size(); ++k) {
        auto& eventWeights = weights[k];
        eventWeights.reserve(matches[k].size());

        double sum = 0.0;

        // Calculate gaussian scores and their sum
        for (const auto& match : matches[k]) {
            const double score = std::exp(-match.second / 4.0);

            eventWeights.push_back({match.first, score});
            sum += score;
        }

        // Divide each score by the sum to obtain rkj
        for (auto& association : eventWeights) {
            association.weight /= sum;
        }
    }

    return weights;
}