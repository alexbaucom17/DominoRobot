#ifndef utils_h
#define utils_h

#include <chrono>
#include <vector>

using ClockTimePoint = std::chrono::time_point<std::chrono::steady_clock>;


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

class Timer
{
  public:
    Timer();
    void reset();
    int dt_us();
    int dt_ms();
    float dt_s();

  private:
    ClockTimePoint prev_time_;
    ClockWrapperBase* clock_;
};


ClockTimePoint get_time()
{
  return ClockFactory::getFactoryInstance()->get_clock()->now();
}

class ClockWrapperBase
{
 public:
  virtual ClockTimePoint now() = 0;
};

class ClockWrapper : public ClockWrapperBase
{
 public:
  ClockTimePoint now() override;
};

class MockClockWrapper : public ClockWrapperBase
{
 public:
  ClockTimePoint now() override;
  void advance_sec(float sec);
  void advance_ms(int ms);
  void advance_us(int us);
 private:
  ClockTimePoint internal_time_;
};

class ClockFactory
{
 public:
  enum class Mode
  {
    NORMAL,
    MOCK
  };

  static ClockFactory* getFactoryInstance();
  void set_mode(ClockFactory::Mode mode);
  ClockWrapperBase* get_clock();

  // Delete copy and assignment constructors
  ClockFactory(ClockFactory const&) = delete;
  ClockFactory& operator= (ClockFactory const&) = delete;
 
 private:
  // Make standard constructor private so it can't be created
  ClockFactory();
  static ClockFactory* factory_instance_;
  std::unique_ptr<ClockWrapperBase> clock_instance_;
  ClockFactory::Mode mode_;

};

#endif
