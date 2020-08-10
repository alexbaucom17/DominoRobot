#include "SmoothTrajectoryGenerator.h"
#include <plog/Log.h>
#include "constants.h"

constexpr d6 = 1/6.0;

SmoothTrajectoryGenerator::SmoothTrajectoryGenerator()
  : currentTrajectory_()
{
    currentTrajectory_.complete_ = false;
}

PVTPoint SmoothTrajectoryGenerator::lookup(float time)
{
    return PVTPoint();
}


// TODO update to handle incomplete trajectory
void SmoothTrajectoryGenerator::generatePointToPointTrajectory(Point initialPoint, Point targetPoint, bool fineMode)
{    
    // Print to logs
    PLOGI.printf("Generating trajectory");
    PLOGI.printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGI.printf("Target point: %s", targetPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Generating trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target point: %s", targetPoint.toString().c_str());

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode);
    currentTrajectory_ = generateTrajectory(mpp);
}

void SmoothTrajectoryGenerator::generateConstVelTrajectory(Point initialPoint, Velocity velocity, float moveTime, bool fineMode)
{
  // TODO - impliment constant velocity version
}

MotionPlanningProblem SmoothTrajectoryGenerator::buildMotionPlanningProblem(Point initialPoint, Point targetPoint, bool fineMode)
{
    MotionPlanningProblem mpp;
    mpp.initialPoint_ = {initialPoint.x_, initialPoint.y_, initialPoint.a_};
    mpp.targetPoint_ = {targetPoint.x_, targetPoint.y_, targetPoint.a_};

    DynamicLimits translationalLimits;
    DynamicLimits rotationalLimits;
    if(fineMode)
    {
        translationalLimits = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_TRANS_JERK_FINE};
        rotationalLimits = {MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE, MAX_ROT_JERK_FINE};
    }
    else
    {
        translationalLimits = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_TRANS_JERK_COARSE};
        rotationalLimits = {MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE, MAX_ROT_JERK_COARSE};
    }

    // This scaling makes sure to give some headroom for the controller to go a bit faster than the planned limits 
    // without actually violating any hard constraints
    mpp.translationalLimits_ = translationalLimits * TRAJ_MAX_FRACTION;
    mpp.rotationalLimits_ = rotationalLimits * TRAJ_MAX_FRACTION; 

    return mpp;     
}

Trajectory SmoothTrajectoryGenerator::generateTrajectory(MotionPlanningProblem problem)
{
    // Figure out delta that the trajectory needs to cover
    Eigen::Vector3f deltaPosition = problem.targetPoint_ - problem.initialPoint_;

    // Being building trajectory object
    Trajectory traj;
    traj.complete_ = false;
    traj.initialPoint_ = problem.initialPoint_;
    traj.direction_ = deltaPosition.normalized();
    
    // Solve translational component
    float dist = deltaPosition.head(2).norm();
    SCurveParameters trans_params;
    bool ok = generateSCurve(dist, problem.translationalLimits_, &trans_params);
    if(!ok)
    {
        PLOGW << "Failed to generate translational trajectory";
        return traj;
    }

    // Solve rotational component
    SCurveParameters rot_params;
    bool ok = generateSCurve(abs(deltaPosition(3)), problem.rotationalLimits_, &rot_params);
    if(!ok)
    {
        PLOGW << "Failed to generate rotational trajectory";
        return traj;
    }

    // TODO: Handle syncronizing between translation and rotational parameters (I think should just be 
    // something like pass in the time steps for the slower one and re-compute the limits)

    return traj;
}

bool SmoothTrajectoryGenerator::generateSCurve(float dist, DynamicLimits limits, SCurveParameters* params)
{
    // Initialize parameters
    params->v_lim_ = limits.max_vel_;
    params->a_lim_ = limits.max_acc_;
    params->j_lim_ = limits.max_jerk_;


    bool solution_found = false;
    int loop_counter = 0;
    while(!solution_found && loop_counter < SOLVER_MAX_LOOPS)
    {
        loop_counter++;

        // Constant jerk region
        float dt_j = params->a_lim / params->j_lim;
        float dv_j = 0.5 * params->j_lim * std::pow(dt_j, 2);
        float dp_j = d6 * params->j_lim * std::pow(dt_j, 3);

        // Constant accel region
        float dt_a = (params->v_lim - 2 * dv_j) / params->a_lim;
        if (dt_a <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust accel parameter and try loop again
            params->a_lim *= SOLVER_BETA_DECAY;
            continue;
        }
        float dp_a = dv_j * dt_a + 0.5 * params->a_lim * std::pow(dt_a, 2);

        // Constant velocity region
        float dt_v = (dist - 4 * dp_j - 2 * dp_a) / params->v_lim;
        if (dt_v <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust velocity parameter and try loop again
            params->v_lim *= SOLVER_ALPHA_DECAY;
            continue;
        }
        float dp_v = params->v_lim * dt_v;

        // If we get here, it means we found a valid solution and can populate the rest of the 
        // switch time parameters
        solution_found = true;
        populateSwitchTimeParameters(params, dt_j, dt_a, dt_v);
    }

    return solution_found;
}

void SmoothTrajectoryGenerator::populateSwitchTimeParameters(SCurveParameters* params, float dt_j, float dt_a, float dt_v)
{
    // Fill first point with all zeros
    params->switch_points_[0].t_ = 0;
    params->switch_points_[0].p_ = 0;
    params->switch_points_[0].v_ = 0;
    params->switch_points_[0].a_ = 0;

    for (int i = 1; i < 8; i++)
    {
        float dt;
        float j;
        // Constant jerk regions
        if (i == 1 || i == 3 || i == 5 || i == 7)
        {
            dt = dt_j;
            // Positive jerk
            if (i == 1 || i == 7) { j = params->j_lim; }
            // Negative jerk
            else { j = -1 * params->j_lim; }
        }
        // Constant acceleration regions
        else if (i == 2 || i == 6)
        {
            dt = dt_a;
            j = 0;
        }
        // Constant velocity region
        else
        {
            dt = dt_v;
            j = 0;
        }

        // Handle integration
        params->switch_points_[i].a_ = params->switch_points_[i-1].a_ + 
                                       j * dt;
        params->switch_points_[i].v_ = params->switch_points_[i-1].v_ + 
                                       params->switch_points_[i].a_ * dt + 
                                       0.5 * j * std::pow(dt, 2);
        params->switch_points_[i].p_ = params->switch_points_[i-1].p_ + 
                                       params->switch_points_[i].v_ * dt + 
                                       0.5 * params->switch_points_[i].a_ * std::pow(dt, 2) + 
                                       d6 * j * std::pow(dt, 3);
    }
}

