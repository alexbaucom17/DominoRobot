#include "SmoothTrajectoryGenerator.h"
#include <plog/Log.h>
#include "constants.h"

constexpr float d6 = 1/6.0;

SmoothTrajectoryGenerator::SmoothTrajectoryGenerator()
  : currentTrajectory_()
{
    currentTrajectory_.complete_ = false;
}

PVTPoint SmoothTrajectoryGenerator::lookup(float time)
{
    return PVTPoint();
}

// TODO: Write some tests for this class

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
    traj.initialPoint_ = {problem.initialPoint_(0), problem.initialPoint_(1), problem.initialPoint_(2)};
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
    ok = generateSCurve(abs(deltaPosition(3)), problem.rotationalLimits_, &rot_params);
    if(!ok)
    {
        PLOGW << "Failed to generate rotational trajectory";
        return traj;
    }

    // Handle syncronizing between translation and rotational parameters so that the 
    // switch points line up with the slower trajectory. This will fail if the difference
    // between the switch times do not all have the same sign (since that means it isn't
    // possible to solve for a set of kinematic parameters that work with the )
    ok = synchronizeParameters(&trans_params, &rot_params);
    if (!ok)
    {
        PLOGW << "Unable to synchronize parameters between translational and rotational trajectories";
    }
    traj.trans_params_ = trans_params;
    traj.rot_params_ = rot_params;
    traj.complete_ = true;

    return traj;
}

bool SmoothTrajectoryGenerator::generateSCurve(float dist, DynamicLimits limits, SCurveParameters* params)
{
    // Initialize parameters
    float v_lim = limits.max_vel_;
    float a_lim = limits.max_acc_;
    float j_lim = limits.max_jerk_;

    bool solution_found = false;
    int loop_counter = 0;
    while(!solution_found && loop_counter < SOLVER_MAX_LOOPS)
    {
        loop_counter++;
        PLOGI << "Trajectory generation loop " << loop_counter;

        // Constant jerk region
        float dt_j = a_lim / j_lim;
        float dv_j = 0.5 * j_lim * std::pow(dt_j, 2);
        float dp_j = d6 * j_lim * std::pow(dt_j, 3);

        // Constant accel region
        float dt_a = (v_lim - 2 * dv_j) / a_lim;
        if (dt_a <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust accel parameter and try loop again
            a_lim *= SOLVER_BETA_DECAY;
            PLOGI << "dt_a: " << dt_a << ", trying new accel value: " << a_lim;
            continue;
        }
        float dp_a = dv_j * dt_a + 0.5 * a_lim * std::pow(dt_a, 2);

        // Constant velocity region
        float dt_v = (dist - 4 * dp_j - 2 * dp_a) / v_lim;
        if (dt_v <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust velocity parameter and try loop again
            v_lim *= SOLVER_ALPHA_DECAY;
            PLOGI << "dt_v: " << dt_v << ", trying new accel value: " << v_lim;
            continue;
        }

        // If we get here, it means we found a valid solution and can populate the rest of the 
        // switch time parameters
        solution_found = true;
        params->v_lim_ = v_lim;
        params->a_lim_ = a_lim;
        params->j_lim_ = j_lim;
        PLOGI << "Trajectory solution found";
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
            if (i == 1 || i == 7) { j = params->j_lim_; }
            // Negative jerk
            else { j = -1 * params->j_lim_; }
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

bool SmoothTrajectoryGenerator::synchronizeParameters(SCurveParameters* params1, SCurveParameters* params2)
{
    int num_positive_diffs = 0;
    int num_negative_diffs = 0;

    for (int i = 0; i < 8; i++)
    {
        float dt = params1->switch_points_[i].t_ - params2[i].switch_points_->t_;
        if(dt > 0)
        {
            num_positive_diffs++;
        }
        else if (dt < 0)
        {
            num_negative_diffs++;
        }
        else
        {
            num_positive_diffs++;
            num_negative_diffs++;
        }
    }

    bool mapping_succeeded = false;
    if(num_positive_diffs == 7)
    {
        // params1 is the slower trajectory, so use it as the reference
        mapping_succeeded = mapParameters(params1, params2);
    }
    else if(num_negative_diffs == 7)
    {
        // params2 is the slower trajectory, so use it as the reference
        mapping_succeeded = mapParameters(params2, params1);
    }
    else
    {
        PLOGW << "Time diffs between trajectories don't lead to feasible synchronization solution";
    }

    // This returns true for successful synchronization
    // This returns false if neither params1 or params2 was definitively slower
    // or if the mapping function fails to solve for a new trajectory
    return mapping_succeeded;
}


bool SmoothTrajectoryGenerator::mapParameters(const SCurveParameters* ref_params, SCurveParameters* map_params)
{
    // Gather parameters needed
    float dt_j = ref_params->switch_points_[1].t_ - ref_params->switch_points_[0].t_;
    float dt_a = ref_params->switch_points_[2].t_ - ref_params->switch_points_[1].t_;
    float dt_v = ref_params->switch_points_[5].t_ - ref_params->switch_points_[3].t_;
    float deltaPosition = map_params->switch_points_[7].p_;

    // Build linear system
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A << dt_j,                                                          -1,                 0    ,
         std::pow(dt_j, 2),                                             dt_a,              -1    ,
         2/3.0 * std::pow(dt_j, 3) + 0.5 * std::pow(dt_j, 2) * dt_a,   std::pow(dt_a, 2),  dt_v;
    b << 0, 0, deltaPosition;

    // Solve system and check results
    Eigen::Vector3f lims = A.colPivHouseholderQr().solve(b);
    float relative_error = (A*lims - b).norm() / b.norm();
    if (relative_error > 1e-5)
    {
        PLOGW << "Could not find feasible inverse parameter mapping";
        return false;
    }

    map_params->j_lim_ = lims(0);
    map_params->a_lim_ = lims(1);
    map_params->v_lim_ = lims(2);
    populateSwitchTimeParameters(map_params, dt_j, dt_a, dt_v);   
    return true;
}