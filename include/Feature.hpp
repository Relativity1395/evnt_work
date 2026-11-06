#pragma once

#include <../third_party/KDTreeVectorOfVectorsAdaptor.h>
#include <../include/Types.hpp>
#include <../include/WeightDist.hpp>


#define MAXDELT 1000000
#define MIN_EVNT_INIT 250
#define MIN_EVNT_DEATH 100
#define MAXITR 50
#define EPSILON 5
#define MAX_WINDOW_EVENTS 400
// Per-feature state / related events tracking
typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2d>, double, 2> my_kd_tree_t;
using Matches = std::vector<nanoflann::ResultItem<size_t, double>>;


const double r2 = 18;
const int dim = 2;
const int leaf = 10;


struct EventGrid {
    static constexpr int CELL = 16;
    int cols, rows;
    std::vector<std::vector<int>> cells;   // indices into currEvents

    EventGrid(int w, int h)
        : cols((w + CELL - 1) / CELL), rows((h + CELL - 1) / CELL), cells(cols * rows) {}

    void build(const std::vector<Event>& E) {
        for (auto& c : cells) c.clear();   // keeps capacity between batches
        for (int i = 0; i < (int)E.size(); i++) {
            int cx = E[i].position.x() / CELL, cy = E[i].position.y() / CELL;
            if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) cells[cy * cols + cx].push_back(i);
        }
    }

    template <class Fn>
    void forEachInBox(double x0, double y0, double x1, double y1, Fn&& fn) const {
        int cx0 = std::max(0, (int)std::floor(x0 / CELL)), cy0 = std::max(0, (int)std::floor(y0 / CELL));
        int cx1 = std::min(cols - 1, (int)std::floor(x1 / CELL)), cy1 = std::min(rows - 1, (int)std::floor(y1 / CELL));
        for (int cy = cy0; cy <= cy1; cy++)
            for (int cx = cx0; cx <= cx1; cx++)
                for (int idx : cells[cy * cols + cx]) fn(idx);
    }
};

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
        int iters = 0;

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
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents();
        void findEvents(const EventGrid& grid, const std::vector<Event>& E);
        bool findEventsInit(const EventGrid& grid, const std::vector<Event>& E);
        bool updateTimeWindow(double highestTimestamp);
        void subsampleEvents(size_t maxN);
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
        double getWindowSize() const { return dti;}
        void assignAlive(bool alive) {health = alive;}
        int setIters(int& iterations) {iters = iterations;}
        bool getHealth() const {return badHealth;}
        int getIters() const {return iters;}
    //3D projection
        void featureProjection(); //takes 2D points and projects them into 3D space

        
};
