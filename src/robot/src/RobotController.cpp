#include "RobotController.h"

#include <math.h>
#include <plog/Log.h>
#include <Eigen/Dense>

#include "constants.h"
#include "serial/SerialCommsFactory.h"

typedef std::chrono::duration<float> fsec;

RobotController::RobotController(StatusUpdater& statusUpdater)
: trajGen_(),
  statusUpdater_(statusUpdater),
  serial_to_motor_driver_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB)),
  prevControlLoopTime_(std::chrono::steady_clock::now()),
  prevOdomLoopTime_(std::chrono::steady_clock::now()),
  trajStartTime_(std::chrono::steady_clock::now()),
  cartPos_(),
  goalPos_(),
  cartVel_(),
  trajRunning_(false),
  fineMode_(true),
  velOnlyMode_(false),
  controller_rate_(cfg.lookup("motion.controller_frequency")),
  logging_rate_(cfg.lookup("motion.log_frequency")),
  log_this_cycle_(false),
  mm_update_fraction_(cfg.lookup("localization.mm_update_fraction")),
  mm_update_vel_fn_slope_(cfg.lookup("localization.mm_update_vel_fn_slope")),
  mm_update_vel_fn_intercept_(cfg.lookup("localization.mm_update_vel_fn_intercept"))
{    
    coarse_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.coarse");
    coarse_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.coarse");
    coarse_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.coarse");
    coarse_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.coarse");

    fine_tolerances_.trans_pos_err = cfg.lookup("motion.translation.position_threshold.fine");
    fine_tolerances_.ang_pos_err = cfg.lookup("motion.rotation.position_threshold.fine");
    fine_tolerances_.trans_vel_err = cfg.lookup("motion.translation.velocity_threshold.fine");
    fine_tolerances_.ang_vel_err = cfg.lookup("motion.rotation.velocity_threshold.fine");
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

void RobotController::moveToPositionRelative(float x, float y, float a)
{
    fineMode_ = false;
    velOnlyMode_ = false;
    goalPos_ = Point(cartPos_.x_ + x, cartPos_.y_ + y, cartPos_.a_ + a);
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
    trajStartTime_ = std::chrono::steady_clock::now();
    prevControlLoopTime_ = std::chrono::steady_clock::now();    

    enableAllMotors();
    PLOGI.printf("Starting move");
}

void RobotController::estop()
{
    PLOGI.printf("Estopping robot control");
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
        cartVel_.vx_ = 0;
        cartVel_.vy_ = 0;
        cartVel_.va_ = 0;

        // Force flags to default values
        fineMode_ = true;
        velOnlyMode_ = false;
    }

    // Run controller and odometry update
    Velocity target_vel = computeControl(cmd);
    setCartVelCommand(target_vel);
    computeOdometry();

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x_, cartPos_.y_, cartPos_.a_);
    statusUpdater_.updateVelocity(cartVel_.vx_, cartVel_.vy_, cartVel_.va_);
}


PVTPoint RobotController::generateCommandFromTrajectory()
{
    std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration_cast<fsec>(curTime - trajStartTime_).count();
    return trajGen_.lookup(dt);
}

PVTPoint RobotController::generateStationaryCommand()
{
    PVTPoint cmd;
    cmd.position_.x_ = cartPos_.x_;
    cmd.position_.y_ = cartPos_.y_;
    cmd.position_.a_ = cartPos_.a_;;
    cmd.velocity_.vx_ = 0;
    cmd.velocity_.vy_ = 0;
    cmd.velocity_.va_ = 0;
    cmd.time_ = 0; // Doesn't matter, not used
    return cmd;
}

Velocity RobotController::computeControl(PVTPoint cmd)
{
    // TODO: Convert back to closed loop when ready
    float x_cmd = cmd.velocity_.vx_;
    float y_cmd = cmd.velocity_.vy_;
    float a_cmd = cmd.velocity_.va_;

    return {x_cmd, y_cmd, a_cmd};
}

bool RobotController::checkForCompletedTrajectory(const PVTPoint cmd)
{
    // Get the right threshold values
    TrajectoryTolerances tol = fineMode_ ? fine_tolerances_ : coarse_tolerances_;

    // Verify our commanded velocity is zero
    bool zeroCmdVel = cmd.velocity_.nearZero();

    // Verify translational and rotational positions are within tolerance
    Eigen::Vector2f dp = {goalPos_.x_ - cartPos_.x_, goalPos_.y_ - cartPos_.y_};
    bool pos_in_tolerance = dp.norm() < tol.trans_pos_err &&
                            fabs(angle_diff(goalPos_.a_, cartPos_.a_)) < tol.ang_pos_err; 

    // Verify translational and rotational velocities are within tolerance
    Eigen::Vector2f dv = {cartVel_.vx_, cartVel_.vy_};
    bool vel_in_tolerance = dv.norm() < tol.trans_vel_err && fabs(cartVel_.va_) < tol.ang_vel_err;

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
        serial_to_motor_driver_->send("Power:ON");
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
        serial_to_motor_driver_->send("Power:OFF");
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
        Eigen::Vector3f v = {cartVel_.vx_, cartVel_.vy_, cartVel_.va_};
        float total_v = v.norm();
        float vel_update_fraction = 1.0;
        if (total_v > 0)
        {
            vel_update_fraction = std::max(mm_update_vel_fn_intercept_ + mm_update_vel_fn_slope_ * total_v, 0.0f);
        }
        float update_fraction = mm_update_fraction_ * vel_update_fraction;
        
        cartPos_.x_ += update_fraction * (x - cartPos_.x_);
        cartPos_.y_ += update_fraction * (y - cartPos_.y_);
        cartPos_.a_ += update_fraction * (a - cartPos_.a_);
    }
}

std::vector<float> RobotController::readMsgFromMotorDriver()
{
    std::string msg = "";
    std::vector<float> decodedVelocity = {0,0,0};
    if (serial_to_motor_driver_->isConnected())
    {
        int count = 0;
        while(msg.empty() && count++ < 10)
        {
            msg = serial_to_motor_driver_->rcv();
        }
    }

    if (msg.empty())
    {
        return {};
    }
    else
    {
        int prev_idx = 0;
        int j = 0;
        for(uint i = 0; i < msg.length(); i++)
        {
            if(msg[i] == ',')
            {
                decodedVelocity[j] = std::stof(msg.substr(prev_idx, i - prev_idx));
                j++;
                prev_idx = i+1;
            }

            if (i == msg.length()-1)
            {
                decodedVelocity[j] = std::stof(msg.substr(prev_idx, std::string::npos));
            }
        }
        if(j != 2)
        {
            PLOGW.printf("Decode failed");
            return {};
        }
    }
    return decodedVelocity;
}

void RobotController::computeOdometry()
{  
 
    std::vector<float> local_cart_vel = readMsgFromMotorDriver();

    if(local_cart_vel.size() != 3) {return;}
    
    std::vector<float> zero = {0,0,0};
    if(trajRunning_ || local_cart_vel != zero)
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("Decoded velocity: %.3f, %.3f, %.3f", local_cart_vel[0], local_cart_vel[1], local_cart_vel[2]);
    }

    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    cartVel_.vx_ = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    cartVel_.vy_ = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    cartVel_.va_ = local_cart_vel[2];

    // Compute time since last odom update
    std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration_cast<fsec>(curTime - prevOdomLoopTime_).count();
    prevOdomLoopTime_ = curTime;

    // Compute new position estimate
    cartPos_.x_ += cartVel_.vx_ * dt;
    cartPos_.y_ += cartVel_.vy_ * dt;
    cartPos_.a_ += cartVel_.va_ * dt;

}

void RobotController::setCartVelCommand(Velocity target_vel)
{
    if (trajRunning_) 
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("CartVelCmd: [vx: %.4f, vy: %.4f, va: %.4f]", target_vel.vx_, target_vel.vy_, target_vel.va_);
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] =  cA * target_vel.vx_ + sA * target_vel.vy_;
    local_cart_vel[1] = -sA * target_vel.vx_ + cA * target_vel.vy_;
    local_cart_vel[2] = target_vel.va_;

    char buff[100];
    sprintf(buff, "%.4f,%.4f,%.4f",local_cart_vel[0], local_cart_vel[1], local_cart_vel[2]);
    std::string s = buff;

    if (local_cart_vel[0] != 0 || local_cart_vel[1] != 0 || local_cart_vel[2] != 0 )
    {
        PLOGD_IF_(MOTION_LOG_ID, log_this_cycle_).printf("Sending to motors: [%s]", s.c_str());
    }

    if (serial_to_motor_driver_->isConnected())
    {
        serial_to_motor_driver_->send(s);
    }
}
