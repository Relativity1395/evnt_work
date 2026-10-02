#pragma once

#include <../third_party/KDTreeVectorOfVectorsAdaptor.h>
#include <../include/Types.hpp>
#include <../include/WeightDist.hpp>
// Per-feature state / related events tracking

class Feature{
    private:
        Eigen::Vector2d position;
        double xi;
        double dti;
        double Ti;
        std::vector<Event> previousEvents;
        std::vector<Event> currentEvents;
        Eigen::Vector2d flow;
        std::vector<Eigen::Vector2d> bProp;
        std::vector<Eigen::Vector2d> landmark;
        std::vector<Eigen::Vector2d> t_flow;
        std::vector<std::vector<Association>> weights;

    public:
        Feature(Eigen::Vector2d position,
                double Ti,
                double dti,
                std::vector<Event> previousEvents,
                std::vector<Event> currentEEvents,
                Eigen::Vector2d flow,
                double xi
            );
        void propagatePreviousEvents();
        const std::vector<Eigen::Vector2d>& getLandmark() const;
        void setEvents();
        void findEvents(const std::vector<Event>& E);
        void updateTimeWindow();
        std::vector<std::vector<nanoflann::ResultItem<size_t, double>>> generateKD();
        Eigen::Vector2d getPosition();
        void updateFeature();
        double getMedianMagnitude();
     
    };
