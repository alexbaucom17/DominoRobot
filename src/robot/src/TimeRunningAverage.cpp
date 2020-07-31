#include "TimeRunningAverage.h"


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