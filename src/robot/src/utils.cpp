#define _USE_MATH_DEFINES
 
#include "utils.h"

#include <cmath>
#include <chrono>
#include <plog/Log.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <plog/Init.h>
#include <plog/Formatters/CsvFormatter.h>
#include <plog/Appenders/RollingFileAppender.h>

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
  dt_us_(1000000 / hz),
  always_ready_(cfg.lookup("motion.rate_always_ready"))
{
}

bool RateController::ready()
{
  if(always_ready_ || timer_.dt_us() > dt_us_)
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
        return;
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


PositionController::PositionController(Gains gains) 
: gains_(gains),
  error_sum_(0.0)
{ }

void PositionController::reset()
{
    error_sum_ = 0.0;
}

float PositionController::compute(float target_position, float actual_position, float target_velocity, float actual_velocity, float dt)
{
    float pos_err = target_position - actual_position;
    float vel_err = target_velocity - actual_velocity;
    error_sum_ += pos_err * dt;
    float output_velocity = target_velocity + gains_.kp * pos_err + gains_.kd * vel_err + gains_.ki * error_sum_;
    return output_velocity;
}


float vectorMean(const std::vector<float>& data)
{
    if(data.empty()) return 0;

    float sum = 0.0;
    for (const auto& val : data)
    {
        sum += val;
    }
    return sum/data.size();
}

float vectorStddev(const std::vector<float>& data, float mean)
{
    if(data.empty()) return 0;
    
    float variance = 0;
    for (const auto& val : data)
    {
        variance += (val - mean) * (val - mean);
    }
    variance /= data.size();
    return sqrt(variance);
}

float zScore(float mean, float stddev, float reading)
{
    if (stddev < 0.0001) return 0;
    return fabs((reading - mean)/stddev);
}


// From: https://www.tutorialspoint.com/parsing-a-comma-delimited-std-string-in-cplusplus
std::vector<std::string> parseCommaDelimitedString(const std::string& str_in)
{
   std::vector<std::string> result;
   std::stringstream s_stream(str_in); //create string stream from the string
   while(s_stream.good()) 
   {
      std::string substr;
      getline(s_stream, substr, ','); //get first string delimited by comma
      result.push_back(substr);
   }
   return result;
}

std::vector<float> parseCommaDelimitedStringToFloat(const std::string& str_in)
{
    std::vector<std::string> str_parsed = parseCommaDelimitedString(str_in);
    std::vector<float> result;
    result.reserve(str_parsed.size());
    for(const auto& val : str_parsed)
    {
        if(val.empty()) continue;
        result.push_back(std::stof(val));
    }
    return result;
}



void reset_last_motion_logger()
{
    // Delete the loggers
    g_logger.reset();
    g_appender.reset();

    // Delete the file
    remove("log/last_motion_log.csv");

    // Re-initialize the logger
    g_appender.reset(new plog::RollingFileAppender<plog::CsvFormatter>("log/last_motion_log.csv"));
    g_logger.reset(new plog::Logger<MOTION_CSV_LOG_ID>(plog::debug));
    g_logger->addAppender(g_appender.get());
}