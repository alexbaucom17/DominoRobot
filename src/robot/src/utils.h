#ifndef utils_h
#define utils_h

#include <chrono>
#include <vector>
#include <memory>

using ClockTimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using FpSeconds = std::chrono::duration<float, std::chrono::seconds::period>;

// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float wrap_angle(float a);

float angle_diff(float a1, float a2);

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
  MockClockWrapper();
  ClockTimePoint now() override;
  void advance_sec(float sec);
  void advance_ms(int ms);
  void advance_us(int us);
  void set(ClockTimePoint time_point);
  void set_now();
 private:
  ClockTimePoint internal_time_;
};


enum class CLOCK_FACTORY_MODE
{
  STANDARD,
  MOCK
};

class ClockFactory
{
 public:
  static ClockFactory* getFactoryInstance();
  void set_mode(CLOCK_FACTORY_MODE mode);
  ClockWrapperBase* get_clock();

  // Delete copy and assignment constructors
  ClockFactory(ClockFactory const&) = delete;
  ClockFactory& operator= (ClockFactory const&) = delete;
 
 private:
  // Make standard constructor private so it can't be created
  ClockFactory();
  void build_clock_instance();
  static ClockFactory* factory_instance_;
  CLOCK_FACTORY_MODE mode_;
  std::unique_ptr<ClockWrapperBase> clock_instance_;
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
    ClockWrapperBase* clock_;
    ClockTimePoint prev_time_;
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
    Timer timer_;

};

class RateController
{
  public:
    RateController(int hz);
    bool ready(); 
  private:
    Timer timer_;
    int dt_us_;
};


#endif
