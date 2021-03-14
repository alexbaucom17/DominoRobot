#ifndef utils_h
#define utils_h

#include <chrono>
#include <vector>
#include <memory>
#include <math.h>

using ClockTimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using FpSeconds = std::chrono::duration<float, std::chrono::seconds::period>;

// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//*******************************************
//           Angle stuff
//*******************************************

// Wrap an angle between +/- 2 pi
float wrap_angle(float a);
// Take diff of two angles, ensures the result is wrapped betwee +/- 2 pi
float angle_diff(float a1, float a2);


//*******************************************
//           Clock/time stuff
//*******************************************

// The ClockWrapperX classes provide wrappers around the system clock to simplified mocking the clock
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

// Provides a way to get the current clock
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


// Simple timer class that can return the time since reset in various formats
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
};


// Keeps an average of how long the time delta is between events
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

// Simple class to run a loop at a certian rate
class RateController
{
  public:
    RateController(int hz);
    bool ready(); 
  private:
    Timer timer_;
    int dt_us_;
    bool always_ready_;
};


//*******************************************
//           Useful data structures
//*******************************************

struct Point
{
    float x;
    float y;
    float a;

    Point(float x=0, float y=0, float a=0)
    : x(x), y(y), a(a)
    {}

    std::string toString() const
    {
        char s[100];
        sprintf(s, "[x: %.3f, y: %.3f, a: %.3f]", x, y, a);
        return static_cast<std::string>(s);
    }

    bool operator== (const Point& other) const
    {
        return x == other.x && y == other.y && a == other.a;
    }
};

struct Velocity
{
    float vx;
    float vy;
    float va;

    Velocity(float vx=0, float vy=0, float va=0)
    : vx(vx), vy(vy), va(va)
    {}

    std::string toString() const
    {
        char s[100];
        sprintf(s, "[vx: %.3f, vy: %.3f, va: %.3f]", vx, vy, va);
        return static_cast<std::string>(s);
    }

    bool operator== (const Velocity& other) const 
    {
        return vx == other.vx && vy == other.vy && va == other.va;
    }

    bool nearZero(float eps=1e-6) const
    {
        if (fabs(vx) < eps && fabs(vy) < eps && fabs(va) < eps)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};


// Very simple circular buffer class
template<class T>
class CircularBuffer 
{
  public:
    CircularBuffer(int size)
    : data_(std::make_unique<std::vector<T>>()),
      size_(size),
      idx_(0),
      full_(false)
    {
        data_->reserve(size_);
    }

    void insert(T val)
    {
        if(!full_) 
        {
            data_->push_back(val);
            idx_++;
            if (idx_ % size_ == 0)
            {
                full_ = true;
                idx_ = 0;
            }
        }
        else 
        {
            data_->at(idx_) = val;
            idx_ = (idx_ + 1) % size_;
        }
    }

    void clear()
    {
        data_->clear();
        idx_ = 0;
        full_ = false;
    }

    std::vector<T> get_contents()
    {
      if(!full_ || idx_ == 0)
      {
        return *data_;
      }
      else 
      {
        std::vector<T> front(data_->begin() + idx_, data_->end());
        std::vector<T> back(data_->begin(), data_->begin() + idx_);
        front.insert(front.end(), back.begin(), back.end());
        return front;
      }
    }

    bool isFull() { return full_;}
    
  private:
    std::unique_ptr<std::vector<T>> data_;
    const int size_;
    int idx_;
    bool full_;

};


class PositionController
{
  public:

    struct Gains
    {
        float kp;
        float ki;
        float kd;    
    };

    PositionController(Gains gains = {0,0,0});

    // Resets error sum to avoid integral windup
    void reset();

    // Compute control velocity for target position with feedforward velocity
    float compute(float target_position, float actual_position, float target_velocity, float actual_velocity, float dt);

  private:

    Gains gains_;
    float error_sum_;
};


//*******************************************
//           Misc math
//*******************************************

float vectorMean(const std::vector<float>& data);

float vectorStddev(const std::vector<float>& data, float mean);

float zScore(float mean, float stddev, float reading);


#endif
