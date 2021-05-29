#include "RobotControllerModeVision.h"
#include "constants.h"
#include <plog/Log.h>
#include "camera_tracker/CameraTrackerFactory.h"

RobotControllerModeVision::RobotControllerModeVision(bool fake_perfect_motion, StatusUpdater& status_updater)
: RobotControllerModeBase(fake_perfect_motion),
  status_updater_(status_updater),
  traj_gen_(),
  goal_point_(0,0,0),
  current_point_(0,0,0),
  current_target_(),
  camera_tracker_(CameraTrackerFactory::getFactoryInstance()->get_camera_tracker()),
  traj_done_timer_(),
  kf_(3,3)
{
    tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.vision");
    tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.vision");
    tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.vision");
    tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.vision");

    PositionController::Gains position_gains;
    position_gains.kp = cfg.lookup("motion.translation.gains_vision.kp");
    position_gains.ki = cfg.lookup("motion.translation.gains_vision.ki");
    position_gains.kd = cfg.lookup("motion.translation.gains_vision.kd");
    x_controller_ = PositionController(position_gains);
    y_controller_ = PositionController(position_gains);

    PositionController::Gains angle_gains;
    angle_gains.kp = cfg.lookup("motion.rotation.gains_vision.kp");
    angle_gains.ki = cfg.lookup("motion.rotation.gains_vision.ki");
    angle_gains.kd = cfg.lookup("motion.rotation.gains_vision.kd");
    a_controller_ = PositionController(angle_gains);

    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf B = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf C = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf Q(3,3);
    Q << cfg.lookup("vision_tracker.kf.predict_trans_cov"),0,0, 
         0,cfg.lookup("vision_tracker.kf.predict_trans_cov"),0,
         0,0,cfg.lookup("vision_tracker.kf.predict_angle_cov");
    Eigen::MatrixXf R(3,3);
    R << cfg.lookup("vision_tracker.kf.meas_trans_cov"),0,0, 
         0,cfg.lookup("vision_tracker.kf.meas_trans_cov"),0,
         0,0,cfg.lookup("vision_tracker.kf.meas_angle_cov");
    kf_ = KalmanFilter(A,B,C,Q,R);
}

bool RobotControllerModeVision::startMove(Point target_point)
{
    CameraTrackerOutput tracker_output = camera_tracker_->getPoseFromCamera();
    if(!tracker_output.ok) 
    {
        PLOGE << "Cannot start vision move, camera pose not ok";
        return false;
    }
    current_point_ = tracker_output.pose;
    goal_point_ = target_point;
    bool ok = traj_gen_.generatePointToPointTrajectory(current_point_, target_point, LIMITS_MODE::VISION);
    if(ok) RobotControllerModeBase::startMove();
    return ok;
}

Velocity RobotControllerModeVision::computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle)
{   
    // Get current target global position and global velocity according to the trajectory
    float dt_from_traj_start = move_start_timer_.dt_s();
    current_target_ = traj_gen_.lookup(dt_from_traj_start);

    // Get previous loop time
    float dt_since_last_loop = loop_timer_.dt_s();
    loop_timer_.reset();

    // Convert global velocity into local robot frame and use it to predict the filter
    Eigen::Vector3f local_vel;
    local_vel[0] = cos(current_position.a) * current_velocity.vx + sin(current_position.a) * current_velocity.vy;
    local_vel[1] = sin(current_position.a) * current_velocity.vx + cos(current_position.a) * current_velocity.vy;
    local_vel[2] = current_velocity.va;
    Eigen::Vector3f udt = local_vel * dt_since_last_loop;
    if(udt.norm() > 0)
    {
        kf_.predict(udt);
    }

    // Get latest pose from cameras and do update step if data is available
    CameraTrackerOutput tracker_output = camera_tracker_->getPoseFromCamera();
    if(tracker_output.ok && tracker_output.timestamp > last_vision_update_time_)
    {
        // Get the current distance measurements from the sensors and update the filter
        Eigen::Vector3f point_vec = {tracker_output.pose.x,tracker_output.pose.y,tracker_output.pose.a};
        kf_.update(point_vec);
        last_vision_update_time_ = tracker_output.timestamp;
    }
    
    // Update current point from state
    Eigen::VectorXf state = kf_.state();
    current_point_ = {state[0], state[1], state[2]};
    status_updater_.updateVisionControllerPose(current_point_);

    // Print motion estimates to log
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "\nTarget: " << current_target_.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est Vel: " << current_velocity.toString();
    PLOGD_IF_(MOTION_LOG_ID, log_this_cycle) << "Est camera pose: " << current_point_.toString();

    // Compute control
    Velocity output_local;
    if(fake_perfect_motion_)
    {
        output_local = current_target_.velocity;
    }
    else
    {
        output_local.vx =  x_controller_.compute(current_target_.position.x, current_point_.x, current_target_.velocity.vx, local_vel[0], dt_since_last_loop);
        output_local.vy =  y_controller_.compute(current_target_.position.y, current_point_.y, current_target_.velocity.vy, local_vel[1], dt_since_last_loop);
        output_local.va =  a_controller_.compute(current_target_.position.a, current_point_.a, current_target_.velocity.va, local_vel[2], dt_since_last_loop);
    }

    // Rotate this commanded velocity back into global frame because currently it is in the local robot frame
    Velocity output_global;
    output_global.vx = cos(current_position.a) * output_local.vx - sin(current_position.a) * output_local.vy;
    output_global.vy = sin(current_position.a) * output_local.vx + cos(current_position.a) * output_local.vy;
    output_global.va = output_local.va;

    PLOGI_(MOTION_CSV_LOG_ID).printf("time,%.4f",dt_from_traj_start);
    PLOGI_(MOTION_CSV_LOG_ID).printf("pos,%.4f,%.4f,%.4f",current_point_.x,current_point_.y,current_point_.a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("target_pos,%.4f,%.4f,%.4f",current_target_.position.x,current_target_.position.y,current_target_.position.a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("vel,%.4f,%.4f,%.4f",local_vel[0],local_vel[1],local_vel[2]);
    PLOGI_(MOTION_CSV_LOG_ID).printf("target_vel,%.4f,%.4f,%.4f",current_target_.velocity.vx,current_target_.velocity.vy,current_target_.velocity.va);
    PLOGI_(MOTION_CSV_LOG_ID).printf("control_vel,%.4f,%.4f,%.4f",output_local.vx,output_local.vy,output_local.va);

    return output_global;
}

bool RobotControllerModeVision::checkForMoveComplete(Point current_position, Velocity current_velocity)
{
    (void) current_position;

    // Verify our target velocity is zero (i.e. we reached the end of the trajectory)
    bool zero_cmd_vel = current_target_.velocity.nearZero();

    // Verify actual translational and rotational positions are within tolerance
    Eigen::Vector2f dp = {goal_point_.x - current_point_.x, goal_point_.y - current_point_.y};
    bool pos_in_tolerance = dp.norm() < tolerances_.trans_pos_err &&
                            fabs(angle_diff(goal_point_.a, current_point_.a)) < tolerances_.ang_pos_err; 

    // Verify actual translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {current_velocity.vx, current_velocity.vy};
    bool vel_in_tolerance = dv.norm() < tolerances_.trans_vel_err && fabs(current_velocity.va) < tolerances_.ang_vel_err;

    // Trajectory is done when we aren't commanding any velocity, our both our actual velocity and
    // position are within the correct tolerance.
    bool maybe_traj_complete = zero_cmd_vel && vel_in_tolerance && pos_in_tolerance;

    if (!maybe_traj_complete) traj_done_timer_.reset();
    else PLOGI_(MOTION_LOG_ID) << "Camera move maybe done";

    if(maybe_traj_complete && traj_done_timer_.dt_s() > 0.8) 
    {
        move_running_ = false;
        PLOGI_(MOTION_LOG_ID) << "Camera move done";
        return true;  
    }

    return false;  
}