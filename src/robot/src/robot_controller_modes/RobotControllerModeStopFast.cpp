#include "RobotControllerModeStopFast.h"
#include "constants.h"
#include <plog/Log.h>

RobotControllerModeStopFast::RobotControllerModeStopFast(bool fake_perfect_motion)
: RobotControllerModeBase(fake_perfect_motion),
  current_target_()
{
    fine_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.fine");
    fine_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.fine");
    fine_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.fine");
    fine_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.fine");

    max_decel_ = {cfg.lookup("motion.translation.max_acc.coarse"),
                  cfg.lookup("motion.translation.max_acc.coarse"),
                  cfg.lookup("motion.rotation.max_acc.coarse")};
                          
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

void RobotControllerModeStopFast::startMove(Point current_position, Velocity current_velocity)
{
    (void) current_position;
    initial_vel_sign_ = {
        sgn(current_velocity.vx),
        sgn(current_velocity.vy),
        sgn(current_velocity.va),
    };
    current_decel_ = {
        -initial_vel_sign_[0] * max_decel_[0],
        -initial_vel_sign_[1] * max_decel_[1],
        -initial_vel_sign_[2] * max_decel_[2],
    };
    PLOGW.printf("Starting STOP_FAST from velocity %s at decel: %f,%f,%f",current_velocity.toString().c_str(),
        current_decel_[0],current_decel_[1], current_decel_[2]);
    RobotControllerModeBase::startMove();
}

Velocity RobotControllerModeStopFast::computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle)
{
    float dt_from_traj_start = move_start_timer_.dt_s();
    float dt = loop_timer_.dt_s();
    Point target_pos = {
        current_position.x + current_velocity.vx * dt + 0.5f * current_decel_[0] * dt * dt,
        current_position.y + current_velocity.vy * dt + 0.5f * current_decel_[1] * dt * dt,
        current_position.a + current_velocity.va * dt + 0.5f * current_decel_[2] * dt * dt,
    };
    Velocity target_vel = {
        current_velocity.vx + current_decel_[0]*dt,
        current_velocity.vy + current_decel_[1]*dt,
        current_velocity.va + current_decel_[2]*dt,
    };

    if(current_velocity.vx != 0 && sgn(current_velocity.vx) != initial_vel_sign_[0]) target_vel.vx = 0;
    if(current_velocity.vy != 0 && sgn(current_velocity.vy) != initial_vel_sign_[1]) target_vel.vy = 0;
    if(current_velocity.va != 0 && sgn(current_velocity.va) != initial_vel_sign_[2]) target_vel.va = 0;

    current_target_ = {target_pos, target_vel, dt_from_traj_start};

    // Print motion estimates to log
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Target: " << current_target_.toString();
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
        output.vx = x_controller_.compute(0, 0, current_target_.velocity.vx, current_velocity.vx, dt_since_last_loop);
        output.vy = y_controller_.compute(0, 0, current_target_.velocity.vy, current_velocity.vy, dt_since_last_loop);
        output.va = a_controller_.compute(0, 0, current_target_.velocity.va, current_velocity.va, dt_since_last_loop);
    }

    PLOGI_(MOTION_CSV_LOG_ID).printf("time,%.4f",dt_from_traj_start);
    PLOGI_(MOTION_CSV_LOG_ID).printf("pos,%.4f,%.4f,%.4f",current_position.x,current_position.y,current_position.a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("target_pos,%.4f,%.4f,%.4f",current_target_.position.x,current_target_.position.y,current_target_.position.a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("vel,%.4f,%.4f,%.4f",current_velocity.vx,current_velocity.vy,current_velocity.va);
    PLOGI_(MOTION_CSV_LOG_ID).printf("target_vel,%.4f,%.4f,%.4f",current_target_.velocity.vx,current_target_.velocity.vy,current_target_.velocity.va);
    PLOGI_(MOTION_CSV_LOG_ID).printf("control_vel,%.4f,%.4f,%.4f",output.vx,output.vy,output.va);

    return output;
}

bool RobotControllerModeStopFast::checkForMoveComplete(Point current_position, Velocity current_velocity)
{
    (void) current_position;
    // Verify our target velocity is zero (i.e. we reached the end of the trajectory)
    bool zero_cmd_vel = current_target_.velocity.nearZero();

    // Verify actual translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {current_velocity.vx, current_velocity.vy};
    bool vel_in_tolerance = dv.norm() < fine_tolerances_.trans_vel_err && fabs(current_velocity.va) < fine_tolerances_.ang_vel_err;

    // Trajectory is done when we aren't commanding any velocity, our both our actual velocity and
    // position are within the correct tolerance.
    bool traj_complete = zero_cmd_vel && vel_in_tolerance;
    
    if(traj_complete) 
    {
        move_running_ = false;
    }

    return traj_complete;  
}