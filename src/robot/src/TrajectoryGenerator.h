#ifndef TrajectoryGenerator_h
#define TrajectoryGenerator_h

#include "constants.h"
#include <plog/Log.h>
#include <vector>

struct Point
{
    float x_;
    float y_;
    float a_;

    Point(float x=0, float y=0, float a=0)
        : x_(x), y_(y), a_(a)
    {}

    void print()
    {
      PLOGI.printf("[X: ");
      PLOGI.printf(x_, 4);
      PLOGI.printf(", Y: ");
      PLOGI.printf(y_, 4);
      PLOGI.printf(", A: ");
      PLOGI.printf(a_, 4);
      PLOGI.printf("]");
    }
};


struct PVTPoint
{
    Point position_;
    Point velocity_;
    float time_;

    void print()
    {
      PLOGI.printf("[Position: ");
      position_.print();
      PLOGI.printf(", Velocity: ");
      velocity_.print();
      PLOGI.printf(", T: ");
      PLOGI.printf(time_, 4);
      PLOGI.printf("]");
    }
};

struct DynamicLimits
{
  float max_trans_vel_;
  float max_trans_acc_;
  float max_rot_vel_;
  float max_rot_acc_;
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
      PLOGI.printf("[p0: ");
      PLOGI.printf(p0_, 4);
      PLOGI.printf(", v0: ");
      PLOGI.printf(v0_, 4);
      PLOGI.printf(", t0: ");
      PLOGI.printf(t0_, 4);
      PLOGI.printf(", a: ");
      PLOGI.printf(a_, 4);
      PLOGI.printf(", tend: ");
      PLOGI.printf(t_end_, 4);
      PLOGI.printf("]");
    }
};


class TrajectoryGenerator
{

    public:

        TrajectoryGenerator();
        void generate(const Point& initialPoint, const Point& targetPoint, const DynamicLimits& limits);
        void generateConstVel(const Point& initialPoint, const float vx, const float vy, const float va, const float t, const DynamicLimits& limits);
        PVTPoint lookup(float time);

    private:

        // Doesn't handle magnitude of 2D trajectories right now, but that
        // can be a future improvement if needed
        struct MultiTrajectory
        {
            std::vector<trajParams> xtraj_;
            std::vector<trajParams> ytraj_;
            std::vector<trajParams> atraj_;

            void print()
            {
              PLOGI.printf("XTRAJ:");
              for(int i = 0; i < xtraj_.size(); i++)
              {
                xtraj_[i].print();
              }
              PLOGI.printf("YTRAJ:");
              for(int i = 0; i < ytraj_.size(); i++)
              {
                ytraj_[i].print();
              }
              PLOGI.printf("ATRAJ:");
              for(int i = 0; i < atraj_.size(); i++)
              {
                atraj_[i].print();
              }
            }
        };
        
        // Helper functions
        std::vector<trajParams> generate_triangle_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<trajParams> generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<trajParams> generate_vel_for_time_1D(float startPos, float vel, float time, float maxAcc) const;
        std::vector<float> lookup_1D(float time, std::vector<trajParams> traj) const;

        MultiTrajectory currentTraj_;


};

#endif
