// Synthetic EM check: an L-shaped corner moving at a known flow, fed in packets.
// EM must recover the flow within 20%, exit before MAXITR, and never kill the feature.
// Usage: ./test_em_convergence [events_per_packet]   (try 12, 60, 600)
#include "OpticalFlow.hpp"
#include <cassert>
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv){
    const size_t P = argc > 1 ? std::atoi(argv[1]) : 600;
    const Eigen::Vector2d u(1e-3, 5e-4);              // px/us (1000, 500 px/s)
    std::vector<Event> all;
    for (uint32_t t = 0; t < 40000; t += 20)
        for (int k = 0; k < 6; k++) {
            Eigen::Vector2d p = Eigen::Vector2d(100, 100) + u * t;
            Eigen::Vector2d q = p + (k < 3 ? Eigen::Vector2d(k*2, 0) : Eigen::Vector2d(0, (k-3)*2));
            Event e; e.position << (uint16_t)std::lround(q.x()), (uint16_t)std::lround(q.y());
            e.timestamp = t; e.polarity = true; all.push_back(e);
        }
    Feature f(Eigen::Vector2d(100,100), 0.0, 0.0, {}, {}, Eigen::Vector2d(0,0), 15.0);
    int maxIters = 0;
    for (size_t s = 0; s < all.size(); s += P) {
        std::vector<Event> pkt(all.begin()+s, all.begin()+std::min(all.size(), s+P));
        assert(FeatureCollection(f, pkt, 1e-3, pkt.back().t()) != DEAD);
        maxIters = std::max(maxIters, f.getIters());
    }
    std::printf("maxIters=%d flow=(%.3e, %.3e) true=(%.3e, %.3e)\n",
                maxIters, f.getFlow().x(), f.getFlow().y(), u.x(), u.y());
    assert(f.hasConverged() && maxIters > 0 && maxIters < 20);
    assert((f.getFlow() - u).norm() < 0.2 * u.norm());
    std::puts("EM convergence test passed");
}
