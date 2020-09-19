#define _USE_MATH_DEFINES
 
#include <cmath>
#include "utils.h"
#include <chrono>

#include <iostream>

float wrap_angle(float a)
{
  while(std::abs(a) > M_PI)
  {
    if(a > M_PI)
    {
      a -= 2*M_PI;
    }
    else if (a < -1*M_PI)
    {
      a += 2*M_PI;
    }
  }
  return a;
}

float angle_diff(float a1, float a2)
{
  float outA = a1 - a2;
  // Handle angle wrapping and compute the correct error amount
  outA = wrap_angle(outA);
  return outA;
}

RateController::RateController(int hz)
: prev_time_(std::chrono::steady_clock::now())
{
  std::chrono::seconds sec(1);
  dt_ = std::chrono::microseconds(sec) / hz;
}

bool RateController::ready()
{
  if(std::chrono::steady_clock::now() - prev_time_ > dt_)
  {
    prev_time_ = std::chrono::steady_clock::now();
    return true;
  }
  return false;
}


TimeRunningAverage::TimeRunningAverage(int window_size)
: buf_(),
  buf_idx_(0),
  window_size_(window_size),
  filled_(false),
  started_(false),
  prev_time_(std::chrono::steady_clock::now())
{
    buf_.reserve(window_size_);
}

int TimeRunningAverage::get_ms()
{
    int sum_end_idx = buf_idx_;
    if(filled_)
    {
        sum_end_idx = window_size_;
    }

    if (sum_end_idx == 0)
    {
        return 0;
    }

    int sum = 0;
    for(int i = 0; i < sum_end_idx; i++)
    {
        sum += buf_[i];
    }
    return sum / sum_end_idx;
}

float TimeRunningAverage::get_sec()
{
    float ms = static_cast<float>(get_ms());
    return ms / 1000.0;
}

void TimeRunningAverage::mark_point()
{

    if(!started_)
    {
        prev_time_ = std::chrono::steady_clock::now();
        started_ = true;
    }

    std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    buf_[buf_idx_] = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - prev_time_).count();
    prev_time_ = std::chrono::steady_clock::now();

    buf_idx_++;
    if (buf_idx_ >= window_size_)
    {
        filled_ = true;
        buf_idx_ = 0;
    }
}