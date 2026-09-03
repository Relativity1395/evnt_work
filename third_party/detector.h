
#ifndef DETECTOR_H
#define DETECTOR_H

#pragma once
#include "timer.h"
#include "../include/dvx_drivers.hpp"

namespace corner_event_detector
{

class Detector {
public:
  Detector(bool connect = true) { (void)connect; }
  virtual ~Detector() = default;
  virtual bool isFeature(const event_t& e) = 0;
protected:
  std::string detector_name_;
};

} // namespace

#endif