#pragma once

#include <vector>
#include <Eigen/Dense>

struct Event{
    Eigen::Vector2d position; // vector for x and y positions
    double timestamp;
    bool polarity;
};

class Feature{
    private:
        Eigen::Vector2d position_;
        std::vector<Event> events_;
        Eigen::Vector2d flow_;
        std::vector<Eigen::Vector2d> landmark_;

    public:
        void propagatePreviousEvents(double Ti);

};



                      