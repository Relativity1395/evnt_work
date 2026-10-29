#pragma once

#include <../third_party/KDTreeVectorOfVectorsAdaptor.h>
#include <../include/Types.hpp>
#include <../include/WeightDist.hpp>
#define MAXDELT 1000000
// Per-feature state / related events tracking
typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;


const double r2 = 4.2426;
const int dim = 2;
const int leaf = 10;
class Feature{


    private:
        Eigen::Vector2d position;
        double xi;
        double dti;
        double Ti;
        double cost = 0.0;
        std::vector<Event> previousEvents;
        std::vector<Event> currentEvents;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> bProp;
        std::vector<Eigen::Vector2d> landmark;
        std::vector<double> t_flow;
        std::vector<std::vector<Association>> weights;
        bool badHealth = false;
        bool converged = false;
        int deadCycles = 0;
        size_t id;
        bool health = false;

    public:

    //Feature tracking
        void updateBackPropagation();
        double getDti() const;
        void updatePosition(double deltaTime);
        Feature(Eigen::Vector2d position,
                double Ti,
                double dti,
                std::vector<Event> currentEvents,
                std::vector<Event> previousEvents,
                Eigen::Vector2d flow,
                double xi
            );
        Eigen::Vector2d getFlow() const; 
        bool isWindowComplete(double highestTimestamp) const;
        void setConverged(bool status); 
        void propagatePreviousEvents();
        bool hasAssociations() const;
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents();
        void findEvents(const std::vector<Event>& E);
        bool findEventsInit(const std::vector<Event>& E);
        bool updateTimeWindow(double highestTimestamp);
        void recordFlow();
        void generateKD(my_kd_tree_t& tree);
        double getCost() const {return cost;}
        Eigen::Vector2d getPosition() const;
        void updateFeature();
        double getMedianMagnitude();
        int checkHealth();
        int getDeadCycles() const;
        double calculateCost();
        Eigen::Vector2d updateFlow();
        bool checkWindow();
        bool checkInitWindow();
        size_t getEventCount() const;
        bool hasConverged() const;
        void setID(size_t newID) { id = newID; }
        size_t getID() const { return id; }
        double getWindowSize() const { return Ti + dti;}
        void assignAlive(bool alive) {health = alive;}
        bool getHealth() const {return badHealth;}
    //3D projection
        void featureProjection(); //takes 2D points and projects them into 3D space

        
};
