#ifndef utils_h
#define utils_h

#include <chrono>
#include <vector>


// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float wrap_angle(float a);

float angle_diff(float a1, float a2);

class RateController
{
  public:
    RateController(int hz);
    bool ready(); 
  private:
    std::chrono::time_point<std::chrono::steady_clock> prev_time_;
    std::chrono::microseconds dt_;
};

class TimeRunningAverage
{
  public:
    TimeRunningAverage(int window_size);

    int get_ms();

    float get_sec();

    void mark_point();

  private:

    std::vector<int> buf_;
    int buf_idx_;
    int window_size_;
    bool filled_;
    bool started_;
    std::chrono::time_point<std::chrono::steady_clock> prev_time_;

};

#endif
