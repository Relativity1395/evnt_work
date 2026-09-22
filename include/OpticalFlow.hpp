#pragma once

#include <vector>
#include <Eigen/Dense>

struct Event{
    Eigen::Vector2d position; // vector for x and y positions
    double timestamp;
    bool polarity;
};

struct Feature{
    private:
        Eigen::Vector2d position;
        std::vector<Event> events;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> landmark;

    public:
        void propagatePreviousEvents(double Ti);

};



                      