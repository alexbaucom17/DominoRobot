#define _USE_MATH_DEFINES
 
#include <cmath>
#include "utils.h"
#include <chrono>
#include <plog/Log.h>

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
: timer_(),
  dt_us_(1000000 / hz)
{
}

bool RateController::ready()
{
  if(timer_.dt_us() > dt_us_)
  {
    timer_.reset();
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
  timer_()
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
        timer_.reset();
        started_ = true;
    }

    buf_[buf_idx_] = timer_.dt_ms();
    timer_.reset();

    buf_idx_++;
    if (buf_idx_ >= window_size_)
    {
        filled_ = true;
        buf_idx_ = 0;
    }
}


Timer::Timer() 
: prev_time_(ClockFactory::getFactoryInstance()->get_clock()->now())
{ }

void Timer::reset()
{
    ClockWrapperBase* clock = ClockFactory::getFactoryInstance()->get_clock();
    prev_time_ = clock->now();
}

int Timer::dt_us()
{
    ClockWrapperBase* clock = ClockFactory::getFactoryInstance()->get_clock();
    return std::chrono::duration_cast<std::chrono::microseconds>(clock->now() - prev_time_).count();
}

int Timer::dt_ms()
{
    ClockWrapperBase* clock = ClockFactory::getFactoryInstance()->get_clock();
    return std::chrono::duration_cast<std::chrono::milliseconds>(clock->now() - prev_time_).count();
}

float Timer::dt_s() 
{
    ClockWrapperBase* clock = ClockFactory::getFactoryInstance()->get_clock();
    return std::chrono::duration_cast<FpSeconds>(clock->now() - prev_time_).count();
}


ClockTimePoint ClockWrapper::now()
{
    return std::chrono::steady_clock::now();
}


MockClockWrapper::MockClockWrapper()
: ClockWrapperBase(),
  internal_time_(std::chrono::steady_clock::now())
{}

ClockTimePoint MockClockWrapper::now()
{
    return internal_time_;
}

void MockClockWrapper::set_now()
{
    set(std::chrono::steady_clock::now());
}

void MockClockWrapper::set(ClockTimePoint time_point)
{
    internal_time_ = time_point;
}

void MockClockWrapper::advance_us(int us)
{
    internal_time_ += std::chrono::microseconds(us);
}

void MockClockWrapper::advance_ms(int ms)
{
    internal_time_ += std::chrono::milliseconds(ms);
}

void MockClockWrapper::advance_sec(float sec)
{
    internal_time_ += std::chrono::duration_cast<std::chrono::microseconds>(FpSeconds(sec));
}



ClockFactory* ClockFactory::factory_instance_ = NULL;

ClockFactory* ClockFactory::getFactoryInstance()
{
    if(!factory_instance_)
    {
        factory_instance_ = new ClockFactory;
    }
    return factory_instance_;
}

void ClockFactory::set_mode(CLOCK_FACTORY_MODE mode)
{
    mode_ = mode;
}

ClockWrapperBase* ClockFactory::get_clock()
{
    if (!clock_instance_)
    {
        build_clock_instance();
    }
    return clock_instance_.get();
}


void ClockFactory::build_clock_instance()
{
    if(mode_ == CLOCK_FACTORY_MODE::STANDARD)
    {
        //PLOGI << "Building STANDARD clock wrapper";
        clock_instance_ = std::make_unique<ClockWrapper>();
    }
    else if (mode_ == CLOCK_FACTORY_MODE::MOCK)
    {
        //PLOGI << "Building MOCK clock wrapper";
        clock_instance_ = std::make_unique<MockClockWrapper>();
    }
}

// Private constructor
ClockFactory::ClockFactory()
: mode_(CLOCK_FACTORY_MODE::STANDARD),
  clock_instance_()
{}
