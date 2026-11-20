
#include <../include/OpticalFlow.hpp>
#include <../include/Feature.hpp>

constexpr double FLOW_TOL_PX = 0.05;
// constexpr double COST_TOL    = epsilo;

std::atomic<int> windowLagWarnings{0};

int FeatureCollection(Feature& F, const EventGrid& grid, const std::vector<Event>& currEvents, const double epsilon, double highestTimestamp){
    if (!F.checkWindow()){
        if (!F.findEventsInit(grid, currEvents)){
            if (highestTimestamp - F.getTi() > MAX_INT_TIME) {
                F.setDeathReason(1);
                F.markDead();
                F.assignAlive(DEAD);
                return DEAD;
            }
            F.assignAlive(INIT);
            return INIT;
        }

        if (F.updateTimeWindow(highestTimestamp)){
            F.setEvents();
            F.findEvents(grid, currEvents);
        }

    } else {
        F.findEvents(grid, currEvents);
    }

    constexpr int MAX_WINDOWS_PER_BATCH = 3;
    int windowsProcessed = 0;

    while (F.isWindowComplete(highestTimestamp) &&
           windowsProcessed < MAX_WINDOWS_PER_BATCH) {

        F.setRawEventCount(F.getEventCount());
        F.subsampleEvents(MAX_WINDOW_EVENTS);
        F.propagatePreviousEvents();

        if (!F.getLandmark().empty()) {
            my_kd_tree_t mat_index(dim, F.getLandmark(), leaf);
            Eigen::Vector2d previousFlow, newFlow;
            double displacementChange = std::numeric_limits<double>::infinity();
            bool validUpdate = false;

            int iter = F.setIters(true);

            do {
                previousFlow = F.getFlow();

                F.updateBackPropagation();
                F.generateKD(mat_index);

                validUpdate = F.updateFlow();

                if (!validUpdate) {
                    break;
                }

                newFlow = F.getFlow();

                displacementChange = (newFlow - previousFlow).norm() * F.getDti();

                iter = F.setIters(false);

            } while (displacementChange > FLOW_TOL_PX && iter < MAXITR);

            F.setFlowChangePx(
                validUpdate ? displacementChange
                            : std::numeric_limits<double>::quiet_NaN()
            );

            F.setConverged(validUpdate && displacementChange <= FLOW_TOL_PX);

            if (F.hasConverged()) {
                F.saveReliableFlow();
            }  // else {
            //     F.restoreReliableFlow();
            // }

            F.calculateCost();
            F.updatePosition(F.getDti());

        } else {
            F.setFlowChangePx(std::numeric_limits<double>::quiet_NaN());
            F.setCost(std::numeric_limits<double>::quiet_NaN());
            F.setIters(true);
            F.setConverged(false);
            F.restoreReliableFlow();
            F.updatePosition(F.getDti());
        }

        int health = F.checkHealth();

        if (health > 3) {
            bool convergenceFailed = !F.hasConverged();
            bool insufficientEvents = F.getEventCount() < MIN_EVNT_DEATH;

            if (convergenceFailed && insufficientEvents)
                F.setDeathReason(4);
            else if (convergenceFailed)
                F.setDeathReason(2);
            else
                F.setDeathReason(3);

            F.assignAlive(DEAD);
        } else {
            F.assignAlive(HEALTHY);
        }

        F.recordWindow();

        if (health > 3) {
            return DEAD;
        }

        windowsProcessed++;

        if (!F.updateTimeWindow(highestTimestamp)) {
            break;
        }

        F.setEvents();
        F.findEvents(grid, currEvents);
    }

    if (F.isWindowComplete(highestTimestamp)) {
        windowLagWarnings.fetch_add(1, std::memory_order_relaxed);
    }

    F.assignAlive(HEALTHY);
    return HEALTHY;
}
