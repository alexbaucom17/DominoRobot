#ifndef QuinticTrajectoryGenerator_h
#define QuinticTrajectoryGenerator_h

#include "Eigen.h"

// Point = [x, y, a] positions
using Point = Eigen::Vector3f;
// Velocity = [vx, vy, va] velocities
using Velocity =Eigen::Vector3f;

// Helper methods to 'stringify' positions and velocities
std::string toString(const Point& p)
{
    char s[100];
    sprintf(s, "[x: %.4f, y: %.4f, a: %.4f]", p[0], p[1], p[2]);
    return static_cast<std::string>(s);
}
std::string toString(const Velcotiy& v)
{
    char s[100];
    sprintf(s, "[vx: %.4f, vy: %.4f, va: %.4f]", v[0], [1], v[2]);
    return static_cast<std::string>(s);
}

struct PVTPoint
{
    Point position_;
    Velcocity velocity_;
    float time_;

    std::string toString() const
    {
      char s[200];
      sprintf(s, "[Position: %s, Velocity: %s, T: %.3f]",position_.toString().c_str(), velocity_.toString().c_str(), time_);
      return static_cast<std::string>(s);
    }

};

struct DynamicLimits
{
  float max_trans_vel_;
  float max_trans_acc_;
  float max_rot_vel_;
  float max_rot_acc_;
};

class QuinticTrajectoryGenerator
{

  public:
    QuinticTrajectoryGenerator();

    void generatePointToPointTrajectory(const Point& initialPoint, const Point& targetPoint, const DynamicLimits& limits);
    void generateConstVelTrajectory(const Point& initialPoint, const Velocity& velocity, const float moveTime, const DynamicLimits& limits);
    PVTPoint lookup(float time);

  private:

    struct Trajectory
    {
        // From http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion
        // q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        Eigen::Vector6f accel_coefficients_;
        Velocity const_vel_;
        Eigen::Vector6f decel_coefficients_;
        // direction = endPoint - startPoint -> super simple "path" to follow
        Eigen::Vector3f direction_;
        PVTPoint initialPoint_;
        std::vector<float> switch_times_; // [start_accel_time, start_cv_time, start_decel_time, end_traj_time ]
    };

    Trajectory currentTrajectory_;

};

#endif