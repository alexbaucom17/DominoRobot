#ifndef TimeRunningAverage_h
#define TimeRunningAverage_h

#include <chrono>
#include <vector>

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