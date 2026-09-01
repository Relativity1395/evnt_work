#pragma once

#include <deque>

#include <libcaer/libcaer.h>
#include "dvx_drivers.hpp"

#include <Eigen/Dense>



namespace corner_event_detector
{

class FastDetector : public Detector
{
public:
  FastDetector(bool connect = true);
  virtual ~FastDetector();

  bool isFeature(event_t e);

private:
  // SAE
  Eigen::MatrixXd sae_[2];

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 640;
  static const int sensor_height_ = 480;
};


} // namespace
