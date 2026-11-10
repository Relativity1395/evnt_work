// Checks: (1) a feature that never gets enough events is dropped after MAX_INIT_TIME,
//         (2) a feature that gets events quickly starts normally,
//         (3) a slow starter's first window is capped.
#include <OpticalFlow.hpp>
#include <cstdio>
#include <random>
static std::vector<Event> burst(int n, double x, double y, uint32_t t0, uint32_t span, std::mt19937& g){
    std::uniform_real_distribution<double> d(-6, 6); std::vector<Event> v;
    for (int i = 0; i < n; i++){ Event e; e.position << (uint16_t)(x + d(g)), (uint16_t)(y + d(g));
        e.timestamp = t0 + (uint32_t)((double)span * i / n); e.polarity = true; v.push_back(e); }
    return v;
}
int run(int perBatch, double& deathTime, double& firstWin, int& lastState){
    std::mt19937 g(1); EventGrid grid(640, 480);
    Feature F(Eigen::Vector2d(100, 100), 0.0, 0.0, {}, {}, Eigen::Vector2d(0, 0), 15.0);
    deathTime = -1; firstWin = 0;
    for (int b = 0; b < 300; b++){                       // 3 s of 10 ms batches
        uint32_t t0 = b * 10000;
        auto ev = burst(perBatch, 100, 100, t0, 10000, g);
        grid.build(ev);
        double hi = t0 + 9999;
        lastState = FeatureCollection(F, grid, ev, 5.0, hi);
        if (firstWin == 0 && F.getWindowSize() > 0) firstWin = F.getWindowSize();
        if (F.getHealth()) { deathTime = hi; break; }
    }
    return 0;
}
int main(){
    double death, win; int st; int fail = 0;
    run(1, death, win, st);     // 1 event per 10 ms: would need 2.5 s to reach 250
    printf("slow feature : died at %.0f us, state %d (DEAD=%d)\n", death, st, DEAD);
    if (!(death > 500000 && death < 520000 && st == DEAD)) { puts("  FAIL: expected removal just after 0.5 s"); fail = 1; }
    run(300, death, win, st);   // plenty of events at once
    printf("fast feature : first window %.0f us, died %.0f\n", win, death);
    if (!(win > 0 && win <= 10000)) { puts("  FAIL: expected a short first window"); fail = 1; }
    run(6, death, win, st);     // reaches 250 events after ~0.42 s
    printf("medium feature: first window %.0f us\n", win);
    if (!(win > 0 && win <= 500000)) { puts("  FAIL: first window should be capped at 0.5 s"); fail = 1; }
    puts(fail ? "RESULT: FAIL" : "RESULT: PASS"); return fail;
}