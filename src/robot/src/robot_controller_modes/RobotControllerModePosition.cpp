#include "RobotControllerModePosition.h"
#include "constants.h"
#include <plog/Log.h>

RobotControllerModePosition::RobotControllerModePosition(bool fake_perfect_motion)
: RobotControllerModeBase(fake_perfect_motion),
  traj_gen_(),
  fine_mode_(true),
  goal_pos_(0,0,0),
  current_target_()
{
    coarse_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.coarse");
    coarse_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.coarse");
    coarse_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.coarse");
    coarse_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.coarse");

    fine_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.fine");
    fine_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.fine");
    fine_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.fine");
    fine_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.fine");

    PositionController::Gains position_gains;
    position_gains.kp = cfg.lookup("motion.translation.gains.kp");
    position_gains.ki = cfg.lookup("motion.translation.gains.ki");
    position_gains.kd = cfg.lookup("motion.translation.gains.kd");
    x_controller_ = PositionController(position_gains);
    y_controller_ = PositionController(position_gains);

    PositionController::Gains angle_gains;
    angle_gains.kp = cfg.lookup("motion.rotation.gains.kp");
    angle_gains.ki = cfg.lookup("motion.rotation.gains.ki");
    angle_gains.kd = cfg.lookup("motion.rotation.gains.kd");
    a_controller_ = PositionController(angle_gains);
}

bool RobotControllerModePosition::startMove(Point current_position, Point target_position, bool fine_mode)
{
    fine_mode_ = fine_mode;
    goal_pos_ = target_position;
    bool ok = traj_gen_.generatePointToPointTrajectory(current_position, target_position, fine_mode_);
    if(ok) RobotControllerModeBase::startMove();
    return ok;
}

Velocity RobotControllerModePosition::computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle)
{
    float dt_from_traj_start = move_start_timer_.dt_s();
    current_target_ = traj_gen_.lookup(dt_from_traj_start);

    // Print motion estimates to log
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "\nTarget: " << current_target_.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est Vel: " << current_velocity.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est Pos: " << current_position.toString();
    
    Velocity output;
    if(fake_perfect_motion_)
    {
        output = current_target_.velocity;
    }
    else 
    {
        float dt_since_last_loop = loop_timer_.dt_s();
        loop_timer_.reset();
        output.vx = x_controller_.compute(current_target_.position.x, current_position.x, current_target_.velocity.vx, current_velocity.vx, dt_since_last_loop);
        output.vy = y_controller_.compute(current_target_.position.y, current_position.y, current_target_.velocity.vy, current_velocity.vy, dt_since_last_loop);
        output.va = a_controller_.compute(current_target_.position.a, current_position.a, current_target_.velocity.va, current_velocity.va, dt_since_last_loop);
    }

    return output;
}

bool RobotControllerModePosition::checkForMoveComplete(Point current_position, Velocity current_velocity)
{
    // Get the right threshold values
    TrajectoryTolerances tol = fine_mode_ ? fine_tolerances_ : coarse_tolerances_;

    // Verify our target velocity is zero (i.e. we reached the end of the trajectory)
    bool zero_cmd_vel = current_target_.velocity.nearZero();

    // Verify actual translational and rotational positions are within tolerance
    Eigen::Vector2f dp = {goal_pos_.x - current_position.x, goal_pos_.y - current_position.y};
    bool pos_in_tolerance = dp.norm() < tol.trans_pos_err &&
                            fabs(angle_diff(goal_pos_.a, current_position.a)) < tol.ang_pos_err; 

    // Verify actual translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {current_velocity.vx, current_velocity.vy};
    bool vel_in_tolerance = dv.norm() < tol.trans_vel_err && fabs(current_velocity.va) < tol.ang_vel_err;

    // Trajectory is done when we aren't commanding any velocity, our both our actual velocity and
    // position are within the correct tolerance.
    bool traj_complete = zero_cmd_vel && vel_in_tolerance && pos_in_tolerance;
    
    if(traj_complete) 
    {
        move_running_ = false;
    }

    return traj_complete;  
}