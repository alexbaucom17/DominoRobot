#ifndef SmoothTrajectoryGenerator_h
#define SmoothTrajectoryGenerator_h

#include <Eigen/Dense>
#include "utils.h"


// Return structure for a trajectory point lookup that contains all the info about a point in time the controller
// needs to drive the robot
struct PVTPoint
{
    Point position;
    Velocity velocity;
    float time;

    std::string toString() const
    {
      char s[200];
      sprintf(s, "[Position: %s, Velocity: %s, T: %.3f]", position.toString().c_str(), velocity.toString().c_str(), time);
      return static_cast<std::string>(s);
    }

};

// Contains info about the maximum dynamic limits of a trajectory
struct DynamicLimits
{
    float max_vel;
    float max_acc;
    float max_jerk;

    DynamicLimits operator* (float c)
    {
        DynamicLimits rtn;
        rtn.max_vel = c * max_vel;
        rtn.max_acc = c * max_acc;
        rtn.max_jerk = c * max_jerk;
        return rtn;
    }
};

// A fully defined point for switching from one region of the trajectory
// to another - needed for efficient lookup without building a huge table
struct SwitchPoint
{
    float t;
    float p;
    float v;
    float a;
};

// Parameters defining a 1-D S-curve trajectory
struct SCurveParameters
{
    float v_lim;
    float a_lim;
    float j_lim;
    SwitchPoint switch_points[8];

    std::string toString() const
    {
      char s[256];
      sprintf(s, "    Limits: [v: %.3f, a: %.3f, j: %.3f]\n    Switch times: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", v_lim, a_lim, j_lim,
      switch_points[0].t,switch_points[1].t,switch_points[2].t,switch_points[3].t,switch_points[4].t,switch_points[5].t,switch_points[6].t,switch_points[7].t);
      return static_cast<std::string>(s);
    }
};

// Everything needed to define a point to point s-curve trajectory in X, Y, and angle
struct Trajectory
{
    Eigen::Vector2f trans_direction;
    int rot_direction;
    Point initialPoint;
    SCurveParameters trans_params;
    SCurveParameters rot_params;
    bool complete;

    std::string toString() const
    {
      char s[1000];
      sprintf(s, "Trajectory Parameters:\nTranslation:\n  Direction: [%.2f, %.2f]\n  S-Curve:\n%s\nRotation:\n  Direction: %i\n  S-Curve:\n%s\n", 
        trans_direction[0], trans_direction[1], trans_params.toString().c_str(), rot_direction, rot_params.toString().c_str());
      return static_cast<std::string>(s);
    }
};

struct SolverParameters
{
    int num_loops;
    float alpha_decay;
    float beta_decay;
    float exponent_decay;
};

enum class LIMITS_MODE
{
    COARSE,
    FINE,
    VISION,
    SLOW,
};

// All the pieces needed to define the motion planning problem
struct MotionPlanningProblem
{
    Eigen::Vector3f initialPoint;
    Eigen::Vector3f targetPoint;
    DynamicLimits translationalLimits;
    DynamicLimits rotationalLimits;  
    SolverParameters solver_params;
};

// Helper methods - making public for easier testing
MotionPlanningProblem buildMotionPlanningProblem(Point initialPoint, Point targetPoint, LIMITS_MODE limits_mode, const SolverParameters& solver);
Trajectory generateTrajectory(MotionPlanningProblem problem);
bool generateSCurve(float dist, DynamicLimits limits, const SolverParameters& solver, SCurveParameters* params);
void populateSwitchTimeParameters(SCurveParameters* params, float dt_j, float dt_a, float dt_v);
bool synchronizeParameters(SCurveParameters* params1, SCurveParameters* params2);
bool slowDownParamsToMatchTime(SCurveParameters* params, float time_to_match);
bool solveInverse(SCurveParameters* params);
std::vector<float> lookup_1D(float time, const SCurveParameters& params);
std::vector<float> computeKinematicsBasedOnRegion(const SCurveParameters& params, int region, float dt);


class SmoothTrajectoryGenerator
{

  public:
    SmoothTrajectoryGenerator();

    // Generates a trajectory that starts at the initial point and ends at the target point. Setting fineMode to true makes the 
    // adjusts the dynamic limits for a more accurate motion. Returns a bool indicating if trajectory generation was
    // successful
    bool generatePointToPointTrajectory(Point initialPoint, Point targetPoint, LIMITS_MODE limits_mode);

    // Generates a trajectory that attempts to maintain the target velocity for a specified time. Note that the current implimentation
    // of this does not give a guarantee on the accuracy of the velocity if the specified velocity and move time would violate the dynamic 
    // limits of the fine or coarse movement mode. Returns a bool indicating if trajectory generation was successful
    bool generateConstVelTrajectory(Point initialPoint, Velocity velocity, float moveTime, LIMITS_MODE limits_mode);

    // Looks up a point in the current trajectory based on the time, in seconds, from the start of the trajectory
    PVTPoint lookup(float time);

  private:

    // The current trajectory - this lets the generation class hold onto this and just provide a lookup method
    // since I don't have a need to pass the trajectory around anywhere
    Trajectory currentTrajectory_;
    
    // These need to be part of the class because they need to be loaded at construction time, not
    // program initialization time (i.e. as globals). This is because the config file is not
    // yet loaded at program start up time.
    SolverParameters solver_params_;
};

#endif