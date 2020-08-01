#ifndef TrajectoryGenerator_h
#define TrajectoryGenerator_h

#include "constants.h"
#include <plog/Log.h>
#include <vector>

//TODO: Convert this to a 'proper' trajectory with vectors and path scaling. Maybe also 5th order?
struct Point
{
    float x_;
    float y_;
    float a_;

    Point(float x=0, float y=0, float a=0)
        : x_(x), y_(y), a_(a)
    {}

    // void print() const
    // {
    //   PLOGI.printf("%s",this->toString().c_str());
    // }

    std::string toString() const
    {
      char s[100];
      sprintf(s, "[X: %.4f, Y: %.4f, A: %.4f]", x_, y_, a_);
      return static_cast<std::string>(s);
    }

};


struct PVTPoint
{
    Point position_;
    Point velocity_;
    float time_;

    std::string toString() const
    {
      char s[200];
      sprintf(s, "[Position: %s, Velocity: %s, T: %.3f]",position_.toString().c_str(), velocity_.toString().c_str(), time_);
      return static_cast<std::string>(s);
    }

    // void print() const
    // {
    //   PLOGI.printf("%s",this->toString().c_str());
    // }
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

    void print() const
    {
      PLOGD_(MOTION_LOG_ID).printf("[p0: %.4f, v0: %.4f, t0: %.4f, a: %.4f, tend: %.4f]",p0_, v0_, t0_, a_, t_end_);
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
              PLOGD_(MOTION_LOG_ID).printf("XTRAJ:");
              for(uint i = 0; i < xtraj_.size(); i++)
              {
                xtraj_[i].print();
              }
              PLOGD_(MOTION_LOG_ID).printf("YTRAJ:");
              for(uint i = 0; i < ytraj_.size(); i++)
              {
                ytraj_[i].print();
              }
              PLOGD_(MOTION_LOG_ID).printf("ATRAJ:");
              for(uint i = 0; i < atraj_.size(); i++)
              {
                atraj_[i].print();
              }
            }
        };
        
        // Helper functions
        std::vector<trajParams> generate_triangle_1D(float startPos, float endPos, float maxAcc) const;
        std::vector<trajParams> generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<trajParams> generate_vel_for_time_1D(float startPos, float vel, float time, float maxAcc) const;
        std::vector<float> lookup_1D(float time, std::vector<trajParams> traj) const;

        MultiTrajectory currentTraj_;


};

#endif
