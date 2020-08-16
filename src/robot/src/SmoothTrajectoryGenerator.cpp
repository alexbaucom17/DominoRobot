#include "SmoothTrajectoryGenerator.h"
#include <plog/Log.h>
#include "constants.h"
#include "utils.h"

constexpr float d6 = 1/6.0;

SmoothTrajectoryGenerator::SmoothTrajectoryGenerator()
  : currentTrajectory_()
{
    currentTrajectory_.complete_ = false;
    solver_params_.num_loops_ = cfg.lookup("trajectory_generation.solver_max_loops"); 
    solver_params_.beta_decay_ = cfg.lookup("trajectory_generation.solver_beta_decay");
    solver_params_.alpha_decay_ = cfg.lookup("trajectory_generation.solver_alpha_decay");
}

// TODO: Write some tests for this class
// TODO: Verify accuracy of final position calculation (since we aren't directly providing it) - mabye do this with a test?
// TODO: Verify that angles work correctly (again maybe a test?) - need to make sure wrapping or unwrapping doesn't get weird

PVTPoint SmoothTrajectoryGenerator::lookup(float time)
{
    std::vector<float> trans_values = lookup_1D(time, currentTrajectory_.trans_params_);
    std::vector<float> rot_values = lookup_1D(time, currentTrajectory_.rot_params_);    

    // Map translational trajectory into XY space with direction vector
    Eigen::Vector2f trans_pos_delta = trans_values[0] * currentTrajectory_.trans_direction_;
    Eigen::Vector2f trans_vel = trans_values[1] * currentTrajectory_.trans_direction_;
    // Map rotational trajectory into angular space with direction
    float rot_pos_delta = rot_values[0] * currentTrajectory_.rot_direction_;
    float rot_vel = rot_values[1] * currentTrajectory_.rot_direction_;

    // Build and return pvtpoint
    PVTPoint pvt;
    pvt.position_ = {currentTrajectory_.initialPoint_.x_ + trans_pos_delta(0),
                     currentTrajectory_.initialPoint_.y_ + trans_pos_delta(1),
                     currentTrajectory_.initialPoint_.a_ + rot_pos_delta };
    pvt.velocity_ = {trans_vel(0), trans_vel(1), rot_vel};
    pvt.time_ = time;
    return pvt;
}

std::vector<float> lookup_1D(float time, const SCurveParameters& params)
{
    // Handle time before start of trajectory
    if(time <= params.switch_points_[0].t_)
    {
        return {params.switch_points_[0].p_, params.switch_points_[0].v_};
    }
    // Handle time after the end of the trajectory
    else if (time > params.switch_points_[7].t_)
    {
        return {params.switch_points_[7].p_, params.switch_points_[7].v_};
    }
    // Handle times within the trajectory
    else
    {
        // Look for correct region
        for (int i = 1; i <= 7; i++)
        {
            // Once region is found, compute position and velocity from previous switch point
            if(params.switch_points_[i-1].t_ < time && time <= params.switch_points_[i].t_)
            {
                float dt = time - params.switch_points_[i-1].t_;
                std::vector<float> values = computeKinematicsBasedOnRegion(params, i, dt);
                return {values[2], values[1]};
            }
        }
    }

    PLOGE << "Should not get to this point in lookup";
    return {0,0};
}


bool SmoothTrajectoryGenerator::generatePointToPointTrajectory(Point initialPoint, Point targetPoint, bool fineMode)
{    
    // Print to logs
    PLOGI.printf("Generating trajectory");
    PLOGI.printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGI.printf("Target point: %s", targetPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Generating trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target point: %s", targetPoint.toString().c_str());

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode, solver_params_);
    currentTrajectory_ = generateTrajectory(mpp);

    return currentTrajectory_.complete_;
}

// TODO Impliment a more accurate version of this if needed
// Note that this implimentation is a hack and isn't guarneteed to give an accurate constant velocity - so use with caution.
bool SmoothTrajectoryGenerator::generateConstVelTrajectory(Point initialPoint, Velocity velocity, float moveTime, bool fineMode)
{
    // Print to logs
    PLOGI.printf("Generating constVel (sort of) trajectory");
    PLOGI.printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGI.printf("Target velocity: %s", velocity.toString().c_str());
    PLOGI.printf("Move time: %d", moveTime);
    PLOGD_(MOTION_LOG_ID).printf("Generating constVel (sort of) trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target velocity: %s", velocity.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Move time: %d", moveTime);

    // This will undershoot the target velocity because we aren't consider accel/jerk here so the 
    // solver will not quite reach this velocity - especially if the move time specified is small.
    Point targetPoint;
    targetPoint.x_ = initialPoint.x_ + velocity.vx_ * moveTime;
    targetPoint.y_ = initialPoint.y_ + velocity.vy_ * moveTime;
    targetPoint.a_ = initialPoint.a_ + velocity.va_ * moveTime;

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode, solver_params_);
    currentTrajectory_ = generateTrajectory(mpp);

    return currentTrajectory_.complete_;
}

MotionPlanningProblem buildMotionPlanningProblem(Point initialPoint, Point targetPoint, bool fineMode, const SolverParameters& solver)
{
    MotionPlanningProblem mpp;
    mpp.initialPoint_ = {initialPoint.x_, initialPoint.y_, initialPoint.a_};
    mpp.targetPoint_ = {targetPoint.x_, targetPoint.y_, targetPoint.a_};

    DynamicLimits translationalLimits;
    DynamicLimits rotationalLimits;
    if(fineMode)
    {
        translationalLimits = { cfg.lookup("motion.translation.max_vel.fine"), 
                                cfg.lookup("motion.translation.max_acc.fine"), 
                                cfg.lookup("motion.translation.max_jerk.fine")};
        rotationalLimits = {    cfg.lookup("motion.rotation.max_vel.fine"), 
                                cfg.lookup("motion.rotation.max_acc.fine"), 
                                cfg.lookup("motion.rotation.max_jerk.fine")};
    }
    else
    {
        translationalLimits = { cfg.lookup("motion.translation.max_vel.coarse"), 
                                cfg.lookup("motion.translation.max_acc.coarse"), 
                                cfg.lookup("motion.translation.max_jerk.coarse")};
        rotationalLimits = {    cfg.lookup("motion.rotation.max_vel.coarse"), 
                                cfg.lookup("motion.rotation.max_acc.coarse"), 
                                cfg.lookup("motion.rotation.max_jerk.coarse")};
    }

    // This scaling makes sure to give some headroom for the controller to go a bit faster than the planned limits 
    // without actually violating any hard constraints
    mpp.translationalLimits_ = translationalLimits * cfg.lookup("motion.limit_max_fraction");
    mpp.rotationalLimits_ = rotationalLimits * cfg.lookup("motion.limit_max_fraction");
    mpp.solver_params_ = solver;

    return std::move(mpp);     
}

Trajectory generateTrajectory(MotionPlanningProblem problem)
{
    // Figure out delta that the trajectory needs to cover
    Eigen::Vector3f deltaPosition = problem.targetPoint_ - problem.initialPoint_;

    // Being building trajectory object
    Trajectory traj;
    traj.complete_ = false;
    traj.initialPoint_ = {problem.initialPoint_(0), problem.initialPoint_(1), problem.initialPoint_(2)};
    traj.trans_direction_ = deltaPosition.head(2).normalized();
    traj.rot_direction_ = sgn(deltaPosition(2));
    
    // Solve translational component
    float dist = deltaPosition.head(2).norm();
    SCurveParameters trans_params;
    bool ok = generateSCurve(dist, problem.translationalLimits_, problem.solver_params_, &trans_params);
    if(!ok)
    {
        PLOGW << "Failed to generate translational trajectory";
        return traj;
    }

    // Solve rotational component
    SCurveParameters rot_params;
    ok = generateSCurve(abs(deltaPosition(2)), problem.rotationalLimits_, problem.solver_params_, &rot_params);
    if(!ok)
    {
        PLOGW << "Failed to generate rotational trajectory";
        return traj;
    }


    // TODO: Fix this, I don't think it works
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

bool generateSCurve(float dist, DynamicLimits limits, const SolverParameters& solver, SCurveParameters* params)
{
    // Handle case where distance is 0
    if (dist == 0)
    {
        params->v_lim_ = 0;
        params->a_lim_ = 0;
        params->j_lim_ = 0;
        for (int i = 0; i < 8; i++)
        {
            params->switch_points_[i].t_ = 0;
            params->switch_points_[i].p_ = 0;
            params->switch_points_[i].v_ = 0;
            params->switch_points_[i].a_ = 0;
        }
    }
    
    
    // Initialize parameters
    float v_lim = limits.max_vel_;
    float a_lim = limits.max_acc_;
    float j_lim = limits.max_jerk_;

    bool solution_found = false;
    int loop_counter = 0;
    while(!solution_found && loop_counter < solver.num_loops_)
    {
        loop_counter++;
        PLOGI << "Trajectory generation loop " << loop_counter;

        // Constant jerk region
        float dt_j = a_lim / j_lim;
        float dv_j = 0.5 * j_lim * std::pow(dt_j, 2);
        float dp_j1 = d6 * j_lim * std::pow(dt_j, 3);
        float dp_j2 = (v_lim - dv_j) * dt_j + 0.5 * a_lim * std::pow(dt_j, 2) - d6 * j_lim * std::pow(dt_j, 3);

        // Constant accel region
        float dt_a = (v_lim - 2 * dv_j) / a_lim;
        if (dt_a <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust accel parameter and try loop again
            a_lim *= solver.beta_decay_;
            PLOGI << "dt_a: " << dt_a << ", trying new accel value: " << a_lim;
            continue;
        }
        float dp_a = dv_j * dt_a + 0.5 * a_lim * std::pow(dt_a, 2);

        // Constant velocity region
        float dt_v = (dist - 2 * dp_j1 - 2 * dp_j2 - 2 * dp_a) / v_lim;
        if (dt_v <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust velocity parameter and try loop again
            v_lim *= solver.alpha_decay_;
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

void populateSwitchTimeParameters(SCurveParameters* params, float dt_j, float dt_a, float dt_v)
{
    // Fill first point with all zeros
    params->switch_points_[0].t_ = 0;
    params->switch_points_[0].p_ = 0;
    params->switch_points_[0].v_ = 0;
    params->switch_points_[0].a_ = 0;

    for (int i = 1; i < 8; i++)
    {
        float dt;
        // Constant jerk regions
        if (i == 1 || i == 3 || i == 5 || i == 7)
        {
            dt = dt_j;
        }
        // Constant acceleration regions
        else if (i == 2 || i == 6)
        {
            dt = dt_a;
        }
        // Constant velocity region
        else
        {
            dt = dt_v;
        }

        // Populate values
        std::vector<float> values = computeKinematicsBasedOnRegion(*params, i, dt);
        params->switch_points_[i].a_ = values[0];
        params->switch_points_[i].v_ = values[1];
        params->switch_points_[i].p_ = values[2];
        params->switch_points_[i].t_ = params->switch_points_[i-1].t_ + dt;
    }
}

bool synchronizeParameters(SCurveParameters* params1, SCurveParameters* params2)
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


bool mapParameters(const SCurveParameters* ref_params, SCurveParameters* map_params)
{
    // Gather parameters needed
    float dt_j = ref_params->switch_points_[1].t_ - ref_params->switch_points_[0].t_;
    float dt_a = ref_params->switch_points_[2].t_ - ref_params->switch_points_[1].t_;
    float dt_v = ref_params->switch_points_[5].t_ - ref_params->switch_points_[3].t_;
    float deltaPosition = map_params->switch_points_[7].p_;

    // Build linear system
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A << dt_j,                      -1,                                     0    ,
         std::pow(dt_j, 2),         dt_a                    ,              -1    ,
         std::pow(dt_j, 2) * dt_a,  std::pow(dt_j, 2) + std::pow(dt_a, 2),  dt_v + 2* dt_j;
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

std::vector<float> computeKinematicsBasedOnRegion(const SCurveParameters& params, int region, float dt)
{
    float j, a, v, p;
    bool need_a = true;
    bool need_v = true;

    // Positive jerk
    if (region == 1 || region == 7) 
    { 
        j = params.j_lim_; 
    }
    // Negative jerk
    else if (region == 3 || region == 5) 
    { 
        j = -1 * params.j_lim_; 
    }
    // Constant positive acceleration
    else if (region == 2)
    {
        j = 0;
        a = params.a_lim_;
        need_a = false;        
    }
    // Constant negative acceleration
    else if ( region == 6)
    {
        j = 0;
        a = -1*params.a_lim_;
        need_a = false; 
    }
    // Constant velocity region
    else if (region == 4)
    {
        j = 0;
        a = 0;
        v = params.v_lim_;
        need_a = false; 
        need_v = false; 
    }
    else
    {
        // Error
        PLOGE << "Invalid region value: " << region;
        return {0,0,0,0};
    }

    // Compute remaining values
    if (need_a)
    {
        a = params.switch_points_[region-1].a_ + j * dt;
    }

    if (need_v)
    {
        v = params.switch_points_[region-1].v_ + 
            params.switch_points_[region-1].a_ * dt + 
            0.5 * j * std::pow(dt, 2);
    }

    p = params.switch_points_[region-1].p_ + 
        params.switch_points_[region-1].v_ * dt + 
        0.5 * params.switch_points_[region-1].a_ * std::pow(dt, 2) + 
        d6 * j * std::pow(dt, 3);

    return {a,v,p};
}