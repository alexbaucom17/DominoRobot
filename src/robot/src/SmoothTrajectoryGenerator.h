#ifndef SmoothTrajectoryGenerator_h
#define SmoothTrajectoryGenerator_h

#include <Eigen/Dense>

struct Point
{
    float x_;
    float y_;
    float a_;

    Point(float x=0, float y=0, float a=0)
    : x_(x), y_(y), a_(a)
    {}

    std::string toString() const
    {
        char s[100];
        sprintf(s, "[x: %.4f, y: %.4f, a: %.4f]", x_, y_, a_);
        return static_cast<std::string>(s);
    }

    bool operator== (const Point& other) const
    {
        return x_ == other.x_ && y_ == other.y_ && a_ == other.a_;
    }
};

struct Velocity
{
    float vx_;
    float vy_;
    float va_;

    Velocity(float vx=0, float vy=0, float va=0)
    : vx_(vx), vy_(vy), va_(va)
    {}

    std::string toString() const
    {
        char s[100];
        sprintf(s, "[vx: %.4f, vy: %.4f, va: %.4f]", vx_, vy_, va_);
        return static_cast<std::string>(s);
    }

    bool operator== (const Velocity& other) const 
    {
        return vx_ == other.vx_ && vy_ == other.vy_ && va_ == other.va_;
    }
};

// Return structure for a trajectory point lookup that contains all the info about a point in time the controller
// needs to drive the robot
struct PVTPoint
{
    Point position_;
    Velocity velocity_;
    float time_;

    std::string toString() const
    {
      char s[200];
      sprintf(s, "[Position: %s, Velocity: %s, T: %.3f]", position_.toString().c_str(), velocity_.toString().c_str(), time_);
      return static_cast<std::string>(s);
    }

};

// Contains info about the maximum dynamic limits of a trajectory
struct DynamicLimits
{
    float max_vel_;
    float max_acc_;
    float max_jerk_;

    DynamicLimits operator* (float c)
    {
        DynamicLimits rtn;
        rtn.max_vel_ = c * max_vel_;
        rtn.max_acc_ = c * max_acc_;
        rtn.max_jerk_ = c * max_jerk_;
        return rtn;
    }
};

// All the pieces needed to define the motion planning problem
struct MotionPlanningProblem
{
    Eigen::Vector3f initialPoint_;
    Eigen::Vector3f targetPoint_;
    DynamicLimits translationalLimits_;
    DynamicLimits rotationalLimits_;  
};

// A fully defined point for switching from one region of the trajectory
// to another - needed for efficient lookup without building a huge table
struct SwitchPoint
{
    float t_;
    float p_;
    float v_;
    float a_;
};

// Parameters defining a 1-D S-curve trajectory
struct SCurveParameters
{
    float v_lim_;
    float a_lim_;
    float j_lim_;
    SwitchPoint switch_points_[8];
};

// Everything needed to define a point to point s-curve trajectory in X, Y, and angle
struct Trajectory
{
    Eigen::Vector2f trans_direction_;
    float rot_direction_;
    Point initialPoint_;
    SCurveParameters trans_params_;
    SCurveParameters rot_params_;
    bool complete_;
};


class SmoothTrajectoryGenerator
{

  public:
    SmoothTrajectoryGenerator();

    // Generates a trajectory that starts at the initial point and ends at the target point. Setting fineMode to true makes the 
    // adjusts the dynamic limits for a more accurate motion. Returns a bool indicating if trajectory generation was
    // successful
    bool generatePointToPointTrajectory(Point initialPoint, Point targetPoint, bool fineMode);

    // Generates a trajectory that attempts to maintain the target velocity for a specified time. Note that the current implimentation
    // of this does not give a guarentee on the accuracy of the velocity if the specified velocity and move time would violate the dynamic 
    // limits of the fine or coarse movement mode. Returns a bool indicating if trajectory generation was successful
    bool generateConstVelTrajectory(Point initialPoint, Velocity velocity, float moveTime, bool fineMode);

    // Looks up a point in the current trajectory based on the time, in seconds, from the start of the trajectory
    PVTPoint lookup(float time);

  private:

    // The current trajectory - this lets the generation class hold onto this and just provide a lookup method
    // since I don't have a need to pass the trajectory around anywhere
    Trajectory currentTrajectory_;
    
    // These need to be part of the class because they need to be loaded at construction time, not
    // program initalization time (i.e. as globals). This is because the config file is not
    // yet loaded at program start up time.
    int solver_max_loops_;
    float solver_beta_decay_;
    float solver_alpha_decay_;

    // Private helper methods 
    MotionPlanningProblem buildMotionPlanningProblem(Point initialPoint, Point targetPoint, bool fineMode);
    Trajectory generateTrajectory(MotionPlanningProblem problem);
    bool generateSCurve(float dist, DynamicLimits limits, SCurveParameters* params);
    void populateSwitchTimeParameters(SCurveParameters* params, float dt_j, float dt_a, float dt_v);
    bool synchronizeParameters(SCurveParameters* params1, SCurveParameters* params2);
    bool mapParameters(const SCurveParameters* ref_traj, SCurveParameters* map_traj);
    std::vector<float> lookup_1D(float time, const SCurveParameters& params);
    std::vector<float> computeKinematicsBasedOnRegion(const SCurveParameters& params, int region, float dt);
};

#endif