#pragma once

#include <../third_party/KDTreeVectorOfVectorsAdaptor.h>
#include <../include/Types.hpp>

// Per-feature state / related events tracking

class Feature{
    private:
        Eigen::Vector2d position;
        double xi;
        std::vector<Event> previousEvents;
        std::vector<Event> currentEvents;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> bProp;
        std::vector<Eigen::Vector2d> landmark;

    public:
        Feature(Eigen::Vector2d position,
                std::vector<Event> previousEvents,
                std::vector<Event> currentEEvents,
                Eigen::Vector2d flow,
                double xi
            );
        void propagatePreviousEvents(double Ti);
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents(const std::vector<Event>& newEvents);
        void findEvents(const std::vector<Event>& E, double Ti, double dti);
        std::vector<std::vector<nanoflann::ResultItem<size_t, double>>> generateKD();
};
