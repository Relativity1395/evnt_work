#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdint>

// The point of this file is to have basic types includable in one place
// No algorithm step should have to import another step just to use one of its structs
// This file should not depend on anything other than libraries defining data types (E.g Eigen)

//pixel coordinates straight from the DVXplorer (x: 0-639, y: 0-479)
//same type as event_t::x / event_t::y in DVX_Drivers.hpp
//Eigen::Vector<T, N> needs Eigen >= 3.4

using PixelCoord = Eigen::Vector<uint16_t,2>;

//Raw camera event. Field types match event_t in DVX_Drivers.hpp, but with Eigen::Vector for position instead of two separate fields
//12 bytes : 4 (x,y) + 4 (timestamp) + 1 (polarity) + 3 (padding). was 32 with doubles.

struct Event{
    PixelCoord position; //pixel x and y
    uint32_t timestamp; //microseconds
    bool polarity; //true = brightness increased

    Eigen::Vector2d pos() const { return position.cast<double>(); }
    double t() const { return static_cast<double>(timestamp); }
};

static_assert(sizeof(Event) == 12, "Event grew, check its field types");

//The math runs in double precision and seconds (flow in pixel per seconds)
//converts at the boundary with these instead of casting inline everywhere

inline Eigen::Vector2d toVector2D(const PixelCoord& p) {
    return p.cast<double>();}
    
inline double toSeconds(u_int32_t timestamp_us) {
    return static_cast<double> (timestamp_us) * 1e-6;}


struct ImuSample{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    double timestamp;
};

// Stores lists of events and imu samples from most recent camera packets
// Use this in algorithms (for event : events ...)
struct RawData {
    std::vector<Event> events;
    std::vector<ImuSample> imu_samples;
};

// Pose?
// state stuff?