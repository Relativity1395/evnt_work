#pragma once

#include <../third_party/KDTreeVectorOfVectorsAdaptor.h>
#include <../include/Types.hpp>
#include <../include/WeightDist.hpp>
#define MAXDELT 1000000
#define MINEVNT 300
// Per-feature state / related events tracking

//k-d tree over a features landmark; built once per window, queried every EM iteration
using LandmarkTree = KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2>;

class Feature{
    private:
        Eigen::Vector2d position;
        double xi;
        double dti;
        double Ti;
        std::vector<Event> previousEvents;
        std::vector<Event> currentEvents;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> landmark;
        //removed bprop, it stored backpropagated events, computed once in findEvents with the old flow.
        std::vector<Eigen::Vector2d> t_flow;
        std::vector<std::vector<Association>> weights;
        std::vector<nanoflann::ResultItem<size_t, double>> matches; //reused radius search buffer

        bool badHealth = false;
        bool converged = false;
        int deadCycles = 0;
        bool dead = false;

    public:
        Feature(Eigen::Vector2d position,
                double Ti,
                double dti,
                std::vector<Event> currentEvents,
                std::vector<Event> previousEvents,
                Eigen::Vector2d flow,
                double xi
            );
        void propagatePreviousEvents();
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents();
        void findEvents(const std::vector<Event>& E);
        bool findEventsInit(const std::vector<Event>& E);
        bool updateTimeWindow(double highestTimestamp);
        void generateKD(const LandmarkTree& tree);
        const Eigen::Vector2d& getPosition() const;
        const Eigen::Vector2d& getFlow() const;
        double getTi() const;
        std::size_t eventCount() const;
        void updateFeature();
        double getMedianMagnitude();
        int checkHealth(double cost, double epsilon);
        int getDeadCycles();
        double calculateCost();
        Eigen::Vector2d updateFlow();
        bool checkWindow() const;
        bool windowClosed(double highestTimestamp) const;
        bool checkInitWindow();
        void killFeature();
        bool getLife() const;
     
    };
