#include "RobotController.h"

#include <math.h>
#include <plog/Log.h>
#include <Eigen/Dense>

#include "constants.h"
#include "serial/SerialCommsFactory.h"
#include "robot_controller_modes/RobotControllerModePosition.h"
#include "robot_controller_modes/RobotControllerModeVision.h"


RobotController::RobotController(StatusUpdater& statusUpdater)
: statusUpdater_(statusUpdater),
  serial_to_motor_driver_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB)),
  localization_(),
  prevOdomLoopTimer_(),
  cartPos_(),
  cartVel_(),
  trajRunning_(false),
  limits_mode_(LIMITS_MODE::FINE),
  controller_rate_(cfg.lookup("motion.controller_frequency")),
  logging_rate_(cfg.lookup("motion.log_frequency")),
  log_this_cycle_(false),
  fake_perfect_motion_(cfg.lookup("motion.fake_perfect_motion")),
  fake_local_cart_vel_(0,0,0),
  max_cart_vel_limit_({cfg.lookup("motion.translation.max_vel.coarse"),
                       cfg.lookup("motion.translation.max_vel.coarse"),
                       cfg.lookup("motion.rotation.max_vel.coarse")}),
  loop_time_averager_(20)
{    
    if(fake_perfect_motion_) PLOGW << "Fake robot motion enabled";
}

void RobotController::moveToPosition(float x, float y, float a)
{
    reset_last_motion_logger();
    limits_mode_ = LIMITS_MODE::COARSE;
    setCartVelLimits(limits_mode_);
    Point goal_pos = Point(x,y,a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("MoveToPosition: %s",goal_pos.toString().c_str());

    auto position_mode = std::make_unique<RobotControllerModePosition>(fake_perfect_motion_);
    bool ok = position_mode->startMove(cartPos_, goal_pos, limits_mode_);
   
    if (ok) 
    { 
        startTraj(); 
        controller_mode_ = std::move(position_mode);
    }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveToPositionRelative(float dx_local, float dy_local, float da_local)
{
    reset_last_motion_logger();
    limits_mode_ = LIMITS_MODE::COARSE;
    setCartVelLimits(limits_mode_);

    float dx_global =  cos(cartPos_.a) * dx_local - sin(cartPos_.a) * dy_local;
    float dy_global =  sin(cartPos_.a) * dx_local + cos(cartPos_.a) * dy_local;
    float da_global = da_local;
    Point goal_pos = Point(cartPos_.x + dx_global, cartPos_.y + dy_global, wrap_angle(cartPos_.a + da_global));
    PLOGI_(MOTION_CSV_LOG_ID).printf("MoveToPositionRelative: %s",goal_pos.toString().c_str());

    auto position_mode = std::make_unique<RobotControllerModePosition>(fake_perfect_motion_);
    bool ok = position_mode->startMove(cartPos_, goal_pos, limits_mode_);
   
    if (ok) 
    { 
        startTraj(); 
        controller_mode_ = std::move(position_mode);
    }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveToPositionRelativeSlow(float dx_local, float dy_local, float da_local)
{
    reset_last_motion_logger();
    limits_mode_ = LIMITS_MODE::SLOW;
    setCartVelLimits(limits_mode_);

    float dx_global =  cos(cartPos_.a) * dx_local - sin(cartPos_.a) * dy_local;
    float dy_global =  sin(cartPos_.a) * dx_local + cos(cartPos_.a) * dy_local;
    float da_global = da_local;
    Point goal_pos = Point(cartPos_.x + dx_global, cartPos_.y + dy_global, wrap_angle(cartPos_.a + da_global));
    PLOGI_(MOTION_CSV_LOG_ID).printf("MoveToPositionRelativeSlow: %s",goal_pos.toString().c_str());

    auto position_mode = std::make_unique<RobotControllerModePosition>(fake_perfect_motion_);
    bool ok = position_mode->startMove(cartPos_, goal_pos, limits_mode_);
   
    if (ok) 
    { 
        startTraj(); 
        controller_mode_ = std::move(position_mode);
    }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    reset_last_motion_logger();
    limits_mode_ = LIMITS_MODE::FINE;
    setCartVelLimits(limits_mode_);
    Point goal_pos = Point(x,y,a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("MoveToPositionFine: %s",goal_pos.toString().c_str());

    auto position_mode = std::make_unique<RobotControllerModePosition>(fake_perfect_motion_);
    bool ok = position_mode->startMove(cartPos_, goal_pos, limits_mode_);
   
    if (ok) 
    { 
        startTraj(); 
        controller_mode_ = std::move(position_mode);
    }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveConstVel(float vx , float vy, float va, float t)
{
    (void) vx;
    (void) vy;
    (void) va;
    (void) t;
    PLOGE << "Not implimented";
}

void RobotController::moveWithVision(float x, float y, float a)
{
    reset_last_motion_logger();
    limits_mode_ = LIMITS_MODE::VISION;
    setCartVelLimits(limits_mode_);
    Point goal = Point(x,y,a);
    PLOGI_(MOTION_CSV_LOG_ID).printf("MoveWithVision: %s",goal.toString().c_str());
    auto vision_mode = std::make_unique<RobotControllerModeVision>(fake_perfect_motion_, statusUpdater_);
    bool ok = vision_mode->startMove(goal);
   
    if (ok) 
    { 
        startTraj(); 
        controller_mode_ = std::move(vision_mode);
    }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::startTraj()
{
    trajRunning_ = true;
    enableAllMotors();
    PLOGI.printf("Starting move");
}

void RobotController::estop()
{
    PLOGW.printf("Estopping robot control");
    PLOGD_(MOTION_LOG_ID) << "\n====ESTOP====\n";
    trajRunning_ = false;
    limits_mode_ = LIMITS_MODE::FINE;
    disableAllMotors();
}

void RobotController::update()
{    
    // Ensures controller runs at approximately constant rate
    if (!controller_rate_.ready()) { return; }
    // Ensures logging doesn't get out of hand
    log_this_cycle_ = logging_rate_.ready();
    
    // Create a command based on the trajectory or not moving
    Velocity target_vel;
    if(trajRunning_)
    {
        target_vel = controller_mode_->computeTargetVelocity(cartPos_, cartVel_, log_this_cycle_);
        // Check if we are finished with the trajectory
        if (controller_mode_->checkForMoveComplete(cartPos_, cartVel_))
        {
            PLOGI.printf("Reached goal");
            PLOGD_(MOTION_LOG_ID) << "Trajectory complete";
            target_vel = {0,0,0};
            disableAllMotors();
            trajRunning_ = false;
            // Bit of a hack for reseting angle covariance after vision motion
            // This avoids weird rotations from error accumlated in the kalman filter
            if (limits_mode_ == LIMITS_MODE::VISION)
            {
                localization_.resetAngleCovariance();
            }
            // Re-enable fine mode at the end of a trajectory
            limits_mode_ = LIMITS_MODE::FINE;
        }
    }
    else
    {       
        // Force zero velocity
        target_vel = {0,0,0};
        
        // Force current velocity to 0
        localization_.forceZeroVelocity();

        // Force flags to default values
        limits_mode_ = LIMITS_MODE::FINE;
    }

    // Run controller and odometry update
    setCartVelCommand(target_vel);
    computeOdometry();

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x, cartPos_.y, cartPos_.a);
    statusUpdater_.updateVelocity(cartVel_.vx, cartVel_.vy, cartVel_.va);
    loop_time_averager_.mark_point();
    statusUpdater_.updateControlLoopTime(loop_time_averager_.get_ms());
    statusUpdater_.updateLocalizationMetrics(localization_.getLocalizationMetrics());
}


void RobotController::enableAllMotors()
{
    if (serial_to_motor_driver_->isConnected())
    {
        serial_to_motor_driver_->send("base:Power:ON");
        PLOGI << "Motors enabled";
        PLOGD_(MOTION_LOG_ID) << "Motors enabled";
    }
    else
    {
        PLOGW << "No connection: Skipping motor enable";
    }
}

void RobotController::disableAllMotors()
{
    if (serial_to_motor_driver_->isConnected())
    {
        serial_to_motor_driver_->send("base:Power:OFF");
        PLOGI << "Motors disabled";
        PLOGD_(MOTION_LOG_ID) << "Motors disabled";
    }
    else
    {
        PLOGW << "No connection: Skipping motor disable";
    }
}

void RobotController::inputPosition(float x, float y, float a)
{
    bool last_mm_used = false;
    if(limits_mode_ == LIMITS_MODE::FINE || limits_mode_ == LIMITS_MODE::COARSE)
    {
        localization_.updatePositionReading({x,y,a});
        cartPos_ = localization_.getPosition();
        last_mm_used = true;
    }
    statusUpdater_.updateLastMarvelmindPose({x,y,a}, last_mm_used);
}

void RobotController::forceSetPosition(float x, float y, float a)
{
    localization_.forceSetPosition({x,y,a});
    cartPos_ = localization_.getPosition();
}

void RobotController::setCartVelLimits(LIMITS_MODE limits_mode)
{
    if(limits_mode == LIMITS_MODE::VISION)
    {
        max_cart_vel_limit_ = { cfg.lookup("motion.translation.max_vel.vision"), 
                                cfg.lookup("motion.translation.max_vel.vision"), 
                                cfg.lookup("motion.rotation.max_vel.vision")};
    }
    else if(limits_mode == LIMITS_MODE::FINE || limits_mode == LIMITS_MODE::SLOW)
    {
        max_cart_vel_limit_ = { cfg.lookup("motion.translation.max_vel.fine"), 
                                cfg.lookup("motion.translation.max_vel.fine"), 
                                cfg.lookup("motion.rotation.max_vel.fine")};
    }
    else
    {
        max_cart_vel_limit_ = { cfg.lookup("motion.translation.max_vel.coarse"), 
                                cfg.lookup("motion.translation.max_vel.coarse"), 
                                cfg.lookup("motion.rotation.max_vel.coarse")};
    }
}

bool RobotController::readMsgFromMotorDriver(Velocity* decodedVelocity)
{
    std::string msg = "";
    std::vector<float> tmpVelocity = {0,0,0};
    if (serial_to_motor_driver_->isConnected())
    {
        int count = 0;
        while(msg.empty() && count++ < 10)
        {
            msg = serial_to_motor_driver_->rcv_base();
        }
    }

    if (msg.empty())
    {
        return false;
    }
    else
    {
        tmpVelocity = parseCommaDelimitedStringToFloat(msg);
        if(tmpVelocity.size() != 3)
        {
            PLOGW.printf("Decode failed");
            return false;
        }
    }
    decodedVelocity->vx = tmpVelocity[0];
    decodedVelocity->vy = tmpVelocity[1];
    decodedVelocity->va = tmpVelocity[2];
    return true;
}

void RobotController::computeOdometry()
{  
    Velocity local_cart_vel = {0,0,0};
    if(fake_perfect_motion_)
    {
        local_cart_vel = fake_local_cart_vel_;
    }
    else if(!readMsgFromMotorDriver(&local_cart_vel)) { return; }
    
    Velocity zero = {0,0,0};
    if(trajRunning_ || !(local_cart_vel == zero))
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("Decoded velocity: %.3f, %.3f, %.3f\n", local_cart_vel.vx, local_cart_vel.vy, local_cart_vel.va);
    }
    // Bypassing motor feedback and just using fake_local_cart_vel_ here may give better performance, but its hard to tell.

    // Compute time since last odom update
    float dt = prevOdomLoopTimer_.dt_s();
    prevOdomLoopTimer_.reset();

    localization_.updateVelocityReading(local_cart_vel, dt);
    cartPos_ = localization_.getPosition();
    cartVel_ = localization_.getVelocity();

}

void RobotController::setCartVelCommand(Velocity target_vel)
{
    if (trajRunning_) 
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("CartVelCmd: [vx: %.4f, vy: %.4f, va: %.4f]", target_vel.vx, target_vel.vy, target_vel.va);
    }

    // Convert input global velocities to local velocities
    Velocity local_cart_vel;
    float cA = cos(cartPos_.a);
    float sA = sin(cartPos_.a);
    local_cart_vel.vx =  cA * target_vel.vx + sA * target_vel.vy;
    local_cart_vel.vy = -sA * target_vel.vx + cA * target_vel.vy;
    local_cart_vel.va = target_vel.va;

    // Cap velocity for safety
    if(fabs(local_cart_vel.vx) > max_cart_vel_limit_.vx) 
    {
        float clamped_vel = sgn(local_cart_vel.vx) * max_cart_vel_limit_.vx;
        PLOGW.printf("Attempted to command X velocity of %.3f m/s, clamping to %.3f m/s", local_cart_vel.vx, clamped_vel);
        PLOGW_(MOTION_LOG_ID).printf("Attempted to command X velocity of %.3f m/s, clamping to %.3f m/s", local_cart_vel.vx, clamped_vel);
        local_cart_vel.vx = clamped_vel;
    }
    if(fabs(local_cart_vel.vy) > max_cart_vel_limit_.vy) 
    {
        float clamped_vel = sgn(local_cart_vel.vy) * max_cart_vel_limit_.vy;
        PLOGW.printf("Attempted to command Y velocity of %.3f m/s, clamping to %.3f m/s", local_cart_vel.vy, clamped_vel);
        PLOGW_(MOTION_LOG_ID).printf("Attempted to command Y velocity of %.3f m/s, clamping to %.3f m/s", local_cart_vel.vy, clamped_vel);
        local_cart_vel.vy = clamped_vel;
    }
    if(fabs(local_cart_vel.va) > max_cart_vel_limit_.va) 
    {
        float clamped_vel = sgn(local_cart_vel.va) * max_cart_vel_limit_.va;
        PLOGW.printf("Attempted to command A velocity of %.3f rad/s, clamping to %.3f rad/s", local_cart_vel.va, clamped_vel);
        PLOGW_(MOTION_LOG_ID).printf("Attempted to command A velocity of %.3f rad/s, clamping to %.3f rad/s", local_cart_vel.va, clamped_vel);
        local_cart_vel.va = clamped_vel;
    }

    // Prep velocity data to send to motor driver
    char buff[100];
    sprintf(buff, "base:%.4f,%.4f,%.4f",local_cart_vel.vx, local_cart_vel.vy, local_cart_vel.va);
    std::string s = buff;

    if (local_cart_vel.vx != 0 || local_cart_vel.vy != 0 || local_cart_vel.va != 0 )
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("Sending to motors: [%s]", s.c_str());
    }

    if(fake_perfect_motion_)
    {
        fake_local_cart_vel_ = local_cart_vel;
    }
    else if (serial_to_motor_driver_->isConnected())
    {
        serial_to_motor_driver_->send(s);
    }
}
