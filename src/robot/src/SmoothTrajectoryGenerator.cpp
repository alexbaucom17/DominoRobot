#include "SmoothTrajectoryGenerator.h"
#include <plog/Log.h>
#include "constants.h"
#include "utils.h"

constexpr float d6 = 1/6.0;

SmoothTrajectoryGenerator::SmoothTrajectoryGenerator()
  : currentTrajectory_()
{
    currentTrajectory_.complete = false;
    solver_params_.num_loops = cfg.lookup("trajectory_generation.solver_max_loops"); 
    solver_params_.beta_decay = cfg.lookup("trajectory_generation.solver_beta_decay");
    solver_params_.alpha_decay = cfg.lookup("trajectory_generation.solver_alpha_decay");
    solver_params_.exponent_decay = cfg.lookup("trajectory_generation.solver_exponent_decay");
}

PVTPoint SmoothTrajectoryGenerator::lookup(float time)
{
    std::vector<float> trans_values = lookup_1D(time, currentTrajectory_.trans_params);
    std::vector<float> rot_values = lookup_1D(time, currentTrajectory_.rot_params);    

    // Map translational trajectory into XY space with direction vector
    Eigen::Vector2f trans_pos_delta = trans_values[0] * currentTrajectory_.trans_direction;
    Eigen::Vector2f trans_vel = trans_values[1] * currentTrajectory_.trans_direction;
    // Map rotational trajectory into angular space with direction
    float rot_pos_delta = rot_values[0] * currentTrajectory_.rot_direction;
    float rot_vel = rot_values[1] * currentTrajectory_.rot_direction;

    // Build and return pvtpoint
    PVTPoint pvt;
    pvt.position = {currentTrajectory_.initialPoint.x + trans_pos_delta(0),
                     currentTrajectory_.initialPoint.y + trans_pos_delta(1),
                     wrap_angle(currentTrajectory_.initialPoint.a + rot_pos_delta) };
    pvt.velocity = {trans_vel(0), trans_vel(1), rot_vel};
    pvt.time = time;
    return pvt;
}

std::vector<float> lookup_1D(float time, const SCurveParameters& params)
{
    // Handle time before start of trajectory
    if(time <= params.switch_points[0].t)
    {
        return {params.switch_points[0].p, params.switch_points[0].v};
    }
    // Handle time after the end of the trajectory
    else if (time > params.switch_points[7].t)
    {
        return {params.switch_points[7].p, params.switch_points[7].v};
    }
    // Handle times within the trajectory
    else
    {
        // Look for correct region
        for (int i = 1; i <= 7; i++)
        {
            // Once region is found, compute position and velocity from previous switch point
            if(params.switch_points[i-1].t < time && time <= params.switch_points[i].t)
            {
                float dt = time - params.switch_points[i-1].t;
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
    PLOGD_(MOTION_LOG_ID).printf("\nGenerating trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target point: %s", targetPoint.toString().c_str());

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode, solver_params_);
    currentTrajectory_ = generateTrajectory(mpp);

    PLOGI << currentTrajectory_.toString();
    PLOGD_(MOTION_LOG_ID) << currentTrajectory_.toString();

    return currentTrajectory_.complete;
}

// TODO Implement a more accurate version of this if needed
// Note that this implimentation is a hack and isn't guaranteed to give an accurate constant velocity - so use with caution.
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
    targetPoint.x = initialPoint.x + velocity.vx * moveTime;
    targetPoint.y = initialPoint.y + velocity.vy * moveTime;
    targetPoint.a = initialPoint.a + velocity.va * moveTime;

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode, solver_params_);
    currentTrajectory_ = generateTrajectory(mpp);

    PLOGI << currentTrajectory_.toString();
    PLOGD_(MOTION_LOG_ID) << currentTrajectory_.toString();

    return currentTrajectory_.complete;
}

MotionPlanningProblem buildMotionPlanningProblem(Point initialPoint, Point targetPoint, bool fineMode, const SolverParameters& solver)
{
    MotionPlanningProblem mpp;
    mpp.initialPoint = {initialPoint.x, initialPoint.y, initialPoint.a};
    mpp.targetPoint = {targetPoint.x, targetPoint.y, targetPoint.a};

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
    mpp.translationalLimits = translationalLimits * cfg.lookup("motion.limit_max_fraction");
    mpp.rotationalLimits = rotationalLimits * cfg.lookup("motion.limit_max_fraction");
    mpp.solver_params = solver;

    return std::move(mpp);     
}

Trajectory generateTrajectory(MotionPlanningProblem problem)
{   
    // Figure out delta that the trajectory needs to cover
    Eigen::Vector3f deltaPosition = problem.targetPoint - problem.initialPoint;
    deltaPosition(2) = wrap_angle(deltaPosition(2));

    // Begin building trajectory object
    Trajectory traj;
    traj.complete = false;
    traj.initialPoint = {problem.initialPoint(0), problem.initialPoint(1), problem.initialPoint(2)};
    traj.trans_direction = deltaPosition.head(2).normalized();
    traj.rot_direction = sgn(deltaPosition(2));
    
    // Solve translational component
    float dist = deltaPosition.head(2).norm();
    SCurveParameters trans_params;
    bool ok = generateSCurve(dist, problem.translationalLimits, problem.solver_params, &trans_params);
    if(!ok)
    {
        PLOGW << "Failed to generate translational trajectory";
        return traj;
    }

    // Solve rotational component
    SCurveParameters rot_params;
    dist = fabs(deltaPosition(2));
    ok = generateSCurve(dist, problem.rotationalLimits, problem.solver_params, &rot_params);
    if(!ok)
    {
        PLOGW << "Failed to generate rotational trajectory";
        return traj;
    }

    traj.trans_params = trans_params;
    traj.rot_params = rot_params;
    traj.complete = true;

    return traj;
}

bool generateSCurve(float dist, DynamicLimits limits, const SolverParameters& solver, SCurveParameters* params)
{
    // Handle case where distance is very close to 0
    float min_dist = cfg.lookup("trajectory_generation.min_dist_limit"); 
    if (fabs(dist) < min_dist)
    {
        params->v_lim = 0;
        params->a_lim = 0;
        params->j_lim = 0;
        for (int i = 0; i < 8; i++)
        {
            params->switch_points[i].t = 0;
            params->switch_points[i].p = 0;
            params->switch_points[i].v = 0;
            params->switch_points[i].a = 0;
        }
        return true;
    }
    
    // Initialize parameters
    float v_lim = limits.max_vel;
    float a_lim = limits.max_acc;
    float j_lim = limits.max_jerk;

    bool solution_found = false;
    int loop_counter = 0;
    while(!solution_found && loop_counter < solver.num_loops)
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
            a_lim *= std::pow(solver.beta_decay, 1 + solver.exponent_decay * loop_counter);
            PLOGI << "dt_a: " << dt_a << ", trying new accel limit: " << a_lim;
            continue;
        }
        float dp_a = dv_j * dt_a + 0.5 * a_lim * std::pow(dt_a, 2);

        // Constant velocity region
        float dt_v = (dist - 2 * dp_j1 - 2 * dp_j2 - 2 * dp_a) / v_lim;
        if (dt_v <= 0)
        {
            // If dt_a is negative, it means we couldn't find a solution
            // so adjust velocity parameter and try loop again
            v_lim *= std::pow(solver.alpha_decay, 1 + solver.exponent_decay * loop_counter);
            PLOGI << "dt_v: " << dt_v << ", trying new velocity limit: " << v_lim;
            continue;
        }

        // If we get here, it means we found a valid solution and can populate the rest of the 
        // switch time parameters
        solution_found = true;
        params->v_lim = v_lim;
        params->a_lim = a_lim;
        params->j_lim = j_lim;
        PLOGI << "Trajectory solution found";
        populateSwitchTimeParameters(params, dt_j, dt_a, dt_v);
    }

    return solution_found;
}

void populateSwitchTimeParameters(SCurveParameters* params, float dt_j, float dt_a, float dt_v)
{
    // Fill first point with all zeros
    params->switch_points[0].t = 0;
    params->switch_points[0].p = 0;
    params->switch_points[0].v = 0;
    params->switch_points[0].a = 0;

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
        params->switch_points[i].a = values[0];
        params->switch_points[i].v = values[1];
        params->switch_points[i].p = values[2];
        params->switch_points[i].t = params->switch_points[i-1].t + dt;
    }
}

// TODO: Figure out if this is actually needed or worth adding back in (or figure out how to make it more than 99% accurate)
// bool synchronizeParameters(SCurveParameters* params1, SCurveParameters* params2)
// {
//     // Make copy so we don't accidentally overwrite anything until we are ready
//     SCurveParameters params1_copy = *params1;
//     SCurveParameters params2_copy = *params2;
    
//     // Find which sections are slowest in each region
//     std::vector<float> longest_dts;
//     for (int i = 1; i < 7; i++)
//     {
//         float dt1 = params1_copy.switch_points[i].t - params1_copy.switch_points[i-1].t;
//         float dt2 = params2_copy.switch_points[i].t - params2_copy.switch_points[i-1].t;
//         if(dt1 > dt2)
//         {
//             longest_dts.push_back(dt1);
//         }
//         else
//         {
//             longest_dts.push_back(dt2);
//         }
//     }

//     // Synchronize times to match up
//     params1_copy.switch_points[0].t = 0;
//     params2_copy.switch_points[0].t = 0;
//     for (int i = 1; i < 7; i++)
//     {
//         params1_copy.switch_points[i].t = params1_copy.switch_points[i-1].t + longest_dts[i-1];
//         params2_copy.switch_points[i].t = params2_copy.switch_points[i-1].t + longest_dts[i-1];
//     }

//     // Solve inverse problem for new time mapping
//     bool inverse_ok1 = solveInverse(&params1_copy);
//     bool inverse_ok2 = solveInverse(&params2_copy);
//     bool sync_ok = inverse_ok1 && inverse_ok2;

//     // Copy data back if the sync was ok
//     if(sync_ok)
//     {
//         *params1 = params1_copy;
//         *params2 = params2_copy;
//     }
//     else
//     {
//         PLOGW << "Time diffs between trajectories don't lead to feasible synchronization solution";
//     }

//     // This returns true for successful synchronization
//     // This returns false if the inverse function fails to solve for a new trajectory
//     return sync_ok;
// }


// bool solveInverse(SCurveParameters* params)
// {
//     // Gather parameters needed
//     const float dt_j = params->switch_points[1].t - params->switch_points[0].t;
//     const float dt_a = params->switch_points[2].t - params->switch_points[1].t;
//     const float dt_v = params->switch_points[4].t - params->switch_points[3].t;
//     const float deltaPosition = params->switch_points[7].p;

//     // Build linear system
//     Eigen::Matrix3f A;
//     Eigen::Vector3f b;
//     A << dt_j,                                     -1,                                     0    ,
//          std::pow(dt_j, 2),                        dt_a                    ,              -1    ,
//          std::pow(dt_j, 2) * (dt_a - 0.5 * dt_j),  std::pow(dt_j, 2) + std::pow(dt_a, 2),  dt_v + 2* dt_j;
//     b << 0, 0, deltaPosition;

//     // Solve system and check results
//     Eigen::Vector3f lims = A.colPivHouseholderQr().solve(b);
//     float relative_error = (A*lims - b).norm() / b.norm();
//     if (relative_error > 1e-5)
//     {
//         PLOGW << "Could not find feasible inverse parameter mapping";
//         return false;
//     }

//     params->j_lim = lims(0);
//     params->a_lim = lims(1);
//     params->v_lim = lims(2);
//     populateSwitchTimeParameters(params, dt_j, dt_a, dt_v);   
//     return true;
// }

std::vector<float> computeKinematicsBasedOnRegion(const SCurveParameters& params, int region, float dt)
{
    float j, a, v, p;
    bool need_a = true;
    bool need_v = true;

    // Positive jerk
    if (region == 1 || region == 7) 
    { 
        j = params.j_lim; 
    }
    // Negative jerk
    else if (region == 3 || region == 5) 
    { 
        j = -1 * params.j_lim; 
    }
    // Constant positive acceleration
    else if (region == 2)
    {
        j = 0;
        a = params.a_lim;
        need_a = false;        
    }
    // Constant negative acceleration
    else if ( region == 6)
    {
        j = 0;
        a = -1*params.a_lim;
        need_a = false; 
    }
    // Constant velocity region
    else if (region == 4)
    {
        j = 0;
        a = 0;
        v = params.v_lim;
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
        a = params.switch_points[region-1].a + j * dt;
    }

    if (need_v)
    {
        v = params.switch_points[region-1].v + 
            params.switch_points[region-1].a * dt + 
            0.5 * j * std::pow(dt, 2);
    }

    p = params.switch_points[region-1].p + 
        params.switch_points[region-1].v * dt + 
        0.5 * params.switch_points[region-1].a * std::pow(dt, 2) + 
        d6 * j * std::pow(dt, 3);

    return {a,v,p};
}