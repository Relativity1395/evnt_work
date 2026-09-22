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
        Eigen::Vector2d position;
        std::vector<Event> events;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> landmark;

    public:
        Feature(Eigen::Vector2d position,
                std::vector<Event> events,
                Eigen::Vector2d flow);
        void propagatePreviousEvents(double Ti);
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents(const std::vector<Event>& newEvents);


};



                      