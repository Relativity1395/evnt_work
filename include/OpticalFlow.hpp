#pragma once

#include <vector>
#include <Eigen/Dense>

struct Event{
    Eigen::Vector2d position; // vector for x and y positions
    double timestamp;
    bool polarity;
};

std::vector<Eigen::Vector2d> propagatePreviousEvent(const std::vector<Event>& Wi_1, 
                                                    const Eigen::Vector2d& Ui_1, 
                                                    double Ti );