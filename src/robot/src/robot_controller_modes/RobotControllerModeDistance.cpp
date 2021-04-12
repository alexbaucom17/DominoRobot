#include "RobotControllerModeDistance.h"
#include "constants.h"
#include <plog/Log.h>
#include "distance_tracker/DistanceTrackerFactory.h"

RobotControllerModeDistance::RobotControllerModeDistance(bool fake_perfect_motion)
: RobotControllerModeBase(fake_perfect_motion),
  traj_gen_(),
  goal_distance_(0,0,0),
  current_distance_(0,0,0),
  current_target_(),
  distance_tracker_(DistanceTrackerFactory::getFactoryInstance()->get_distance_tracker()),
  move_started_for_real_(false),
  move_start_delay_sec_(1.0),
  traj_done_timer_(),
  kf_(3,3)
{
    distance_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.fine");
    distance_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.fine");
    distance_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.fine");
    distance_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.fine");

    PositionController::Gains position_gains;
    position_gains.kp = cfg.lookup("motion.translation.gains_distance.kp");
    position_gains.ki = cfg.lookup("motion.translation.gains_distance.ki");
    position_gains.kd = cfg.lookup("motion.translation.gains_distance.kd");
    x_controller_ = PositionController(position_gains);
    y_controller_ = PositionController(position_gains);

    PositionController::Gains angle_gains;
    angle_gains.kp = cfg.lookup("motion.rotation.gains_distance.kp");
    angle_gains.ki = cfg.lookup("motion.rotation.gains_distance.ki");
    angle_gains.kd = cfg.lookup("motion.rotation.gains_distance.kd");
    a_controller_ = PositionController(angle_gains);

    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf B = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf C = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf Q(3,3);
    Q << cfg.lookup("distance_tracker.kf.predict_trans_cov"),0,0, 
         0,cfg.lookup("distance_tracker.kf.predict_trans_cov"),0,
         0,0,cfg.lookup("distance_tracker.kf.predict_angle_cov");
    Eigen::MatrixXf R(3,3);
    R << cfg.lookup("distance_tracker.kf.meas_trans_cov"),0,0, 
         0,cfg.lookup("distance_tracker.kf.meas_trans_cov"),0,
         0,0,cfg.lookup("distance_tracker.kf.meas_angle_cov");
    kf_ = KalmanFilter(A,B,C,Q,R);
}

bool RobotControllerModeDistance::startMove(Point goal_distance)
{
    // We need to wait for a few seconds for the distance measurements to populate
    // So we will 'start' the move, but not actually start moving for a short period of time.
    goal_distance_ = goal_distance;
    distance_tracker_->start();
    traj_done_timer_.reset();
    RobotControllerModeBase::startMove();
    return true;
}

void RobotControllerModeDistance::actuallyStartTheMove()
{
    current_distance_ = distance_tracker_->getDistancePose();
    move_started_for_real_ = traj_gen_.generatePointToPointTrajectory(current_distance_, goal_distance_, /*fine_mode=*/true);
    if(!move_started_for_real_) PLOGE << "Failed to start distance move";
}

Velocity RobotControllerModeDistance::computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle)
{    
    // Check if we should actually start the move or not
    if(move_start_timer_.dt_s() < move_start_delay_sec_) return {0,0,0};
    else if (!move_started_for_real_) actuallyStartTheMove();
    
    // Get current target global position and global velocity according to the trajectory
    float dt_from_traj_start = move_start_timer_.dt_s() - move_start_delay_sec_;
    PVTPoint current_target_ = traj_gen_.lookup(dt_from_traj_start);

    // Get previous loop time
    float dt_since_last_loop = loop_timer_.dt_s();
    loop_timer_.reset();

    // Convert global velocity into local relative velocity and use it to predict the filter
    // Need to flip the velocity sign because the distance frame is techncially flipped
    Eigen::Vector3f local_vel;
    local_vel[0] = -1 * (cos(current_position.a) * current_velocity.vx + sin(current_position.a) * current_velocity.vy);
    local_vel[1] = -1 * (sin(current_position.a) * current_velocity.vx + cos(current_position.a) * current_velocity.vy);
    local_vel[2] = current_velocity.va;
    Eigen::Vector3f udt = local_vel * dt_since_last_loop;
    if(udt.norm() > 0)
    {
        kf_.predict(udt);
    }

    // Get the current distance measurements from the sensors and update the filter
    Point measured_distance = distance_tracker_->getDistancePose();
    Eigen::Vector3f dist_vec = {measured_distance.x,measured_distance.y,measured_distance.a};
    kf_.update(dist_vec);
    Eigen::VectorXf state = kf_.state();
    current_distance_ = {state[0], state[1], state[2]};

    // Print motion estimates to log
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "\nTarget: " << current_target_.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est Vel: " << current_velocity.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est Dist: " << current_distance_.toString();

    // Compute control
    Velocity output_local;
    if(fake_perfect_motion_)
    {
        output_local = current_target_.velocity;
    }
    else
    {
        output_local.vx =  x_controller_.compute(current_target_.position.x, current_distance_.x, current_target_.velocity.vx, local_vel[0], dt_since_last_loop);
        output_local.vy =  y_controller_.compute(current_target_.position.y, current_distance_.y, current_target_.velocity.vy, local_vel[1], dt_since_last_loop);
        output_local.va =  a_controller_.compute(current_target_.position.a, current_distance_.a, current_target_.velocity.va, local_vel[2], dt_since_last_loop);
    }

    // Need to flip the velocity sign because the distance frame is techncially flipped
    // Also need to rotate this into global frame because currently it is in the local robot frame
    Velocity output_global;
    output_global.vx = -1 * (cos(current_position.a) * output_local.vx - sin(current_position.a) * output_local.vy);
    output_global.vy = -1 * (sin(current_position.a) * output_local.vx + cos(current_position.a) * output_local.vy);
    output_global.va = output_local.va;

    return output_global;
}

bool RobotControllerModeDistance::checkForMoveComplete(Point current_position, Velocity current_velocity)
{
    (void) current_position;

    // Verify our target velocity is zero (i.e. we reached the end of the trajectory)
    bool zero_cmd_vel = current_target_.velocity.nearZero();

    // Verify actual translational and rotational positions are within tolerance
    Eigen::Vector2f dp = {goal_distance_.x - current_distance_.x, goal_distance_.y - current_distance_.y};
    bool pos_in_tolerance = dp.norm() < distance_tolerances_.trans_pos_err &&
                            fabs(angle_diff(goal_distance_.a, current_distance_.a)) < distance_tolerances_.ang_pos_err; 

    // Verify actual translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {current_velocity.vx, current_velocity.vy};
    bool vel_in_tolerance = dv.norm() < distance_tolerances_.trans_vel_err && fabs(current_velocity.va) < distance_tolerances_.ang_vel_err;

    // Trajectory is done when we aren't commanding any velocity, our both our actual velocity and
    // position are within the correct tolerance.
    bool maybe_traj_complete = zero_cmd_vel && vel_in_tolerance && pos_in_tolerance;

    if (!maybe_traj_complete) traj_done_timer_.reset();
    else PLOGI_(MOTION_LOG_ID) << "Dist move maybe done";

    if(maybe_traj_complete && traj_done_timer_.dt_s() > 0.5) 
    {
        move_running_ = false;
        PLOGI_(MOTION_LOG_ID) << "Dist move done";
        distance_tracker_->stop();
        return true;  
    }

    return false;  
}