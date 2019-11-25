#ifndef TrajectoryGenerator_h
#define TrajectoryGenerator_h

#include "globals.h"
#include <ArduinoSTL.h>
#include <HardwareSerial.h>

struct Point
{
    float x_;
    float y_;
    float a_;

    Point(float x=0, float y=0, float a=0)
        : x_(x), y_(y), a_(a)
    {}

    Point& operator=(const Point& p)
    {
        x_ = p.x_;
        y_ = p.y_;
        a_ = p.a_;
        return *this;
    }

    Point& operator+(const Point& p)
    {
        x_ += p.x_;
        y_ += p.y_;
        a_ += p.a_;
        return *this;
    }

    Point& operator-(const Point& p)
    {
        x_ -= p.x_;
        y_ -= p.y_;
        a_ -= p.a_;
        return *this;
    }

    void print(HardwareSerial& debug)
    {
      debug.print("[X: ");
      debug.print(x_);
      debug.print(", Y: ");
      debug.print(y_);
      debug.print(", A: ");
      debug.print(a_);
      debug.print("]");
    }
};


struct PVTPoint
{
    Point position_;
    Point velocity_;
    float time_;

    void print(HardwareSerial& debug)
    {
      debug.print("[Position: ");
      position_.print(debug);
      debug.print(", Velocity: ");
      velocity_.print(debug);
      debug.print(", T: ");
      debug.print(time_);
      debug.print("]");
    }
};

// Parameters for a 1D constant acceleration trajectory
struct trajParams
{
    float p0_;    // Start position
    float v0_;    // Start velocity
    float t0_;    // Start time
    float a_;     // Const acceleration
    float t_end_; // End time

    void print(HardwareSerial& debug)
    {
      debug.print("[p0: ");
      debug.print(p0_);
      debug.print(", v0: ");
      debug.print(v0_);
      debug.print(", t0: ");
      debug.print(t0_);
      debug.print(", a: ");
      debug.print(a_);
      debug.print(", tend: ");
      debug.print(t_end_);
      debug.print("]");
    }
};


class TrajectoryGenerator
{

    public:

        TrajectoryGenerator(HardwareSerial& debug);
        void generate(const Point& initialPoint, const Point& targetPoint);
        PVTPoint lookup(float time);

    private:

        // Doesn't handle magnitude of 2D trajectories right now, but that
        // can be a future improvement if needed
        struct MultiTrajectory
        {
            std::vector<trajParams> xtraj_;
            std::vector<trajParams> ytraj_;
            std::vector<trajParams> atraj_;

            void print(HardwareSerial& debug)
            {
              debug.println("XTRAJ:");
              for(int i = 0; i < xtraj_.size(); i++)
              {
                xtraj_[i].print(debug);
                debug.println("");
              }
              debug.println("YTRAJ:");
              for(int i = 0; i < ytraj_.size(); i++)
              {
                ytraj_[i].print(debug);
                debug.println("");
              }
              debug.println("ATRAJ:");
              for(int i = 0; i < atraj_.size(); i++)
              {
                atraj_[i].print(debug);
                debug.println("");
              }
            }
        };
        
        // Helper functions
        std::vector<trajParams> generate_triangle_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<trajParams> generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<float> lookup_1D(float time, std::vector<trajParams> traj) const;

        MultiTrajectory currentTraj_;
        const float timeForConstVelTrans_;
        const float posForConstVelTrans_;
        const float timeForConstVelRot_;
        const float posForConstVelRot_;
        HardwareSerial& debug_;


};

#endif
