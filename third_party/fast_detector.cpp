#include "fast_detector.h"

#include <algorithm>


namespace corner_event_detector
{

namespace
{
template <int N, int MIN_STREAK, int MAX_STREAK>
bool hasStreak(const uint32_t* r)

{
  for (int i = 0; i < N; i++)
  {
    // first streak event must no be older than its left neightbor
    if (r[i] < r[i +N -1])
      continue;
    
    for (int streak_size = MIN_STREAK; streak_size <= MAX_STREAK; streak_size++)
    {
      //last streak event must not be older than its right neighbor
      if (r[i + streak_size -1] < r[i+streak_size])
        continue;
      uint32_t min_t = r[i];
      for (int j = 1; j < streak_size; j++)
      {
        min_t = std::min(min_t, r[i +j]);
      }
      bool did_break = false;
      for (int j = streak_size ; j < N; j++)
      {
        if (r[i +j] >= min_t)
        {
          did_break = true;
          break;
        }
      }
      if (!did_break)
      {
        return true;
      }
    }
  }
  return false;
}
}


FastDetector::FastDetector(bool connect)
: Detector(connect),
  circle3_ {{0, 3}, {1, 3}, {2, 2}, {3, 1},
            {3, 0}, {3, -1}, {2, -2}, {1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
            {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
  circle4_ {{0, 4}, {1, 4}, {2, 3}, {3, 2},
            {4, 1}, {4, 0}, {4, -1}, {3, -2},
            {2, -3}, {1, -4}, {0, -4}, {-1, -4},
            {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
            {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}
{
  detector_name_ = "FAST";

  // allocate SAE matrices 
  sae_[0].assign(sensor_width_ * sensor_height_, 0);
  sae_[1].assign(sensor_width_ * sensor_height_, 0);
}

FastDetector::~FastDetector()
{
}

bool FastDetector::isFeature(const Event& e)
{
  // update SAE
  const int ex = static_cast<int>(e.position.x());
  const int ey = static_cast<int>(e.position.y());

  const int pol = e.polarity ? 1 : 0;
  const uint32_t* center = sae_[pol].data() + ey * sensor_width_ + ex;
  sae_[pol][ey * sensor_width_ + ex] = e.timestamp;

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*4;
  if (ex < cs || ex >= sensor_width_-cs ||
      ey < cs || ey >= sensor_height_-cs)
  {
    return false;
  }

  uint32_t r3[32];
  for (int i =0 ; i < 16; i++){
    r3[i] = r3[i +16] = center[circle3_[i][1] * sensor_width_ + circle3_[i][0]];
  }
  if (!hasStreak<16,3,6>(r3))
  {
    return false;
  }
  uint32_t r4[40];
  for (int i =0 ; i < 20; i++){
    r4[i] = r4[i +20] = center[circle4_[i][1] * sensor_width_ + circle4_[i][0]];
  }
  return hasStreak<20,4,8>(r4);
}
} // namespace
