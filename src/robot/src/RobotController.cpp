#include "RobotController.h"

#include <math.h>
#include <plog/Log.h>
#include <Eigen/Dense>

#include "constants.h"
#include "serial/SerialCommsFactory.h"


RobotController::RobotController(StatusUpdater& statusUpdater)
: trajGen_(),
  statusUpdater_(statusUpdater),
  serial_to_motor_driver_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB)),
  localization_(),
  prevControlLoopTimer_(),
  prevOdomLoopTimer_(),
  trajStartTimer_(),
  cartPos_(),
  goalPos_(),
  cartVel_(),
  trajRunning_(false),
  fineMode_(true),
  velOnlyMode_(false),
  controller_rate_(cfg.lookup("motion.controller_frequency")),
  logging_rate_(cfg.lookup("motion.log_frequency")),
  log_this_cycle_(false),
  fake_perfect_motion_(cfg.lookup("motion.fake_perfect_motion")),
  fake_local_cart_vel_(0,0,0),
  loop_time_averager_(20)
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

    if(fake_perfect_motion_) PLOGW << "Fake robot motion enabled";
}

void RobotController::moveToPosition(float x, float y, float a)
{
    fineMode_ = false;
    velOnlyMode_ = false;
    goalPos_ = Point(x,y,a);
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveToPositionRelative(float dx_local, float dy_local, float da_local)
{
    fineMode_ = false;
    velOnlyMode_ = false;
    float dx_global =  cos(cartPos_.a) * dx_local - sin(cartPos_.a) * dy_local;
    float dy_global =  sin(cartPos_.a) * dx_local + cos(cartPos_.a) * dy_local;
    float da_global = da_local;
    goalPos_ = Point(cartPos_.x + dx_global, cartPos_.y + dy_global, wrap_angle(cartPos_.a + da_global));
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    fineMode_ = true;
    velOnlyMode_ = false;
    goalPos_ = Point(x,y,a);
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::moveConstVel(float vx , float vy, float va, float t)
{
    fineMode_ = false;
    velOnlyMode_ = true;
    bool ok = trajGen_.generateConstVelTrajectory(cartPos_, {vx, vy, va}, t, fineMode_);
    if (ok) { startTraj(); }
    else { statusUpdater_.setErrorStatus(); }
}

void RobotController::startTraj()
{
    trajRunning_ = true;
    trajStartTimer_.reset();
    prevControlLoopTimer_.reset();    

    enableAllMotors();
    PLOGI.printf("Starting move");
}

void RobotController::estop()
{
    PLOGW.printf("Estopping robot control");
    PLOGD_(MOTION_LOG_ID) << "\n====ESTOP====\n";
    trajRunning_ = false;
    fineMode_ = true;
    velOnlyMode_ = false;
    disableAllMotors();
}

void RobotController::update()
{    
    // Ensures controller runs at approximately constant rate
    if (!controller_rate_.ready()) { return; }
    // Ensures logging doesn't get out of hand
    log_this_cycle_ = logging_rate_.ready();
    
    // Create a command based on the trajectory or not moving
    PVTPoint cmd;
    if(trajRunning_)
    {
        cmd = generateCommandFromTrajectory();
        
        // Print motion estimates to log
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_) << "\nTarget: " << cmd.toString();
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_) << "Est Vel: " << cartVel_.toString();
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_) << "Est Pos: " << cartPos_.toString();

        // Check if we are finished with the trajectory
        if (checkForCompletedTrajectory(cmd))
        {
            PLOGI.printf("Reached goal");
            PLOGD_(MOTION_LOG_ID) << "Trajectory complete";
            disableAllMotors();
            trajRunning_ = false;
            // Re-enable fine mode at the end of a trajectory
            fineMode_ = true;
        }
    }
    else
    {       
        cmd = generateStationaryCommand();
        
        // Force current velocity to 0
        localization_.forceZeroVelocity();

        x_controller_.reset();
        y_controller_.reset();
        a_controller_.reset();

        // Force flags to default values
        fineMode_ = true;
        velOnlyMode_ = false;
    }

    // Run controller and odometry update
    Velocity target_vel = computeControl(cmd);
    setCartVelCommand(target_vel);
    computeOdometry();

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x, cartPos_.y, cartPos_.a);
    statusUpdater_.updateVelocity(cartVel_.vx, cartVel_.vy, cartVel_.va);
    loop_time_averager_.mark_point();
    statusUpdater_.updateControlLoopTime(loop_time_averager_.get_ms());
    statusUpdater_.updateLocalizationMetrics(localization_.getLocalizationMetrics());
}


PVTPoint RobotController::generateCommandFromTrajectory()
{
    float dt = trajStartTimer_.dt_s();
    return trajGen_.lookup(dt);
}

PVTPoint RobotController::generateStationaryCommand()
{
    PVTPoint cmd;
    cmd.position.x = cartPos_.x;
    cmd.position.y = cartPos_.y;
    cmd.position.a = cartPos_.a;;
    cmd.velocity.vx = 0;
    cmd.velocity.vy = 0;
    cmd.velocity.va = 0;
    cmd.time = 0; // Doesn't matter, not used
    return cmd;
}

Velocity RobotController::computeControl(PVTPoint cmd)
{
    if(fake_perfect_motion_)
    {
        return cmd.velocity;
    }
    
    float dt = prevControlLoopTimer_.dt_s();
    prevControlLoopTimer_.reset();
    
    float x_cmd_vel = x_controller_.compute(cmd.position.x, cartPos_.x, cmd.velocity.vx, cartVel_.vx, dt);
    float y_cmd_vel = y_controller_.compute(cmd.position.y, cartPos_.y, cmd.velocity.vy, cartVel_.vy, dt);
    float a_cmd_vel = a_controller_.compute(cmd.position.a, cartPos_.a, cmd.velocity.va, cartVel_.va, dt);

    return {x_cmd_vel, y_cmd_vel, a_cmd_vel};
}

bool RobotController::checkForCompletedTrajectory(const PVTPoint cmd)
{
    // Get the right threshold values
    TrajectoryTolerances tol = fineMode_ ? fine_tolerances_ : coarse_tolerances_;

    // Verify our commanded velocity is zero
    bool zeroCmdVel = cmd.velocity.nearZero();

    // Verify translational and rotational positions are within tolerance
    Eigen::Vector2f dp = {goalPos_.x - cartPos_.x, goalPos_.y - cartPos_.y};
    bool pos_in_tolerance = dp.norm() < tol.trans_pos_err &&
                            fabs(angle_diff(goalPos_.a, cartPos_.a)) < tol.ang_pos_err; 

    // Verify translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {cartVel_.vx, cartVel_.vy};
    bool vel_in_tolerance = dv.norm() < tol.trans_vel_err && fabs(cartVel_.va) < tol.ang_vel_err;

    // Trajectory is done when we aren't commanding any velocity, our actual velocity is
    // within the correct tolerance and we are either within position tolerance or in velocity
    // only mode where we don't care about the final position.
    bool trajComplete = zeroCmdVel && vel_in_tolerance && (velOnlyMode_ || pos_in_tolerance);

    return trajComplete;   
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
    if (fineMode_)
    {
        localization_.updatePositionReading({x,y,a});
        cartPos_ = localization_.getPosition();
    }
}

void RobotController::forceSetPosition(float x, float y, float a)
{
    localization_.forceSetPosition({x,y,a});
    cartPos_ = localization_.getPosition();
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
        int prev_idx = 0;
        int j = 0;
        for(uint i = 0; i < msg.length(); i++)
        {
            if(msg[i] == ',')
            {
                tmpVelocity[j] = std::stof(msg.substr(prev_idx, i - prev_idx));
                j++;
                prev_idx = i+1;
            }

            if (i == msg.length()-1)
            {
                tmpVelocity[j] = std::stof(msg.substr(prev_idx, std::string::npos));
            }
        }
        if(j != 2)
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
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("Decoded velocity: %.3f, %.3f, %.3f", local_cart_vel.vx, local_cart_vel.vy, local_cart_vel.va);
    }

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
