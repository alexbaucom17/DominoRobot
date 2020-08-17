#include "RobotController.h"

#include <math.h>
#include <plog/Log.h>

#include "constants.h"
#include "utils.h"

typedef std::chrono::duration<float> fsec;

RobotController::RobotController(StatusUpdater& statusUpdater)
: trajGen_(),
  statusUpdater_(statusUpdater),
  serial_to_motor_driver_(buildSerialComms(CLEARCORE_USB)),
  prevControlLoopTime_(std::chrono::steady_clock::now()),
  prevOdomLoopTime_(std::chrono::steady_clock::now()),
  trajStartTime_(std::chrono::steady_clock::now()),
  cartPos_(),
  goalPos_(),
  cartVel_(),
  enabled_(false),
  trajRunning_(false),
  fineMode_(true),
  velOnlyMode_(false)
{    
}

// TODO: Add tests for this class

void RobotController::moveToPosition(float x, float y, float a)
{
    fineMode_ = false;
    velOnlyMode_ = false;
    goalPos_ = Point(x,y,a);
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
}

void RobotController::moveToPositionRelative(float x, float y, float a)
{
    fineMode_ = false;
    velOnlyMode_ = false;
    goalPos_ = Point(cartPos_.x_ + x, cartPos_.y_ + y, cartPos_.a_ + a);
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    fineMode_ = true;
    velOnlyMode_ = false;
    goalPos_ = Point(x,y,a);
    bool ok = trajGen_.generatePointToPointTrajectory(cartPos_, goalPos_, fineMode_);
    if (ok) { startTraj(); }
}

void RobotController::moveConstVel(float vx , float vy, float va, float t)
{
    fineMode_ = false;
    velOnlyMode_ = true;
    bool ok = trajGen_.generateConstVelTrajectory(cartPos_, {vx, vy, va}, t, fineMode_);
    if (ok) { startTraj(); }
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
    // Create a command based on the trajectory or not moving
    PVTPoint cmd;
    if(trajRunning_)
    {
        cmd = generateCommandFromTrajectory();
        
        // Print motion estimates to log
        PLOGD_(MOTION_LOG_ID) << "Target: " << cmd.toString();
        PLOGD_(MOTION_LOG_ID) << "Est Vel: " << cartVel_.toString();
        PLOGD_(MOTION_LOG_ID) << "Est Pos: " << cartPos_.toString();

        // Check if we are finished with the trajectory
        if (checkForCompletedTrajectory(cmd))
        {
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

        // Force flagst to default values
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
    
    // Recompute the loop update time to get better accuracy
    // std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    // float dt = std::chrono::duration_cast<fsec>(curTime - prevControlLoopTime_).count()
    // prevControlLoopTime_ = curTime;

    // // x control 
    // float posErrX = cmd.position_.x_ - cartPos_.x_;
    // float velErrX = cmd.velocity_.x_ - cartVel_.x_;
    // errSumX_ += posErrX * dt;
    // float x_cmd = cmd.velocity_.x_ + CART_TRANS_KP * posErrX + CART_TRANS_KD * velErrX + CART_TRANS_KI * errSumX_;

    // // y control 
    // float posErrY = cmd.position_.y_ - cartPos_.y_;
    // float velErrY = cmd.velocity_.y_ - cartVel_.y_;
    // errSumY_ += posErrY * dt;
    // float y_cmd = cmd.velocity_.y_ + CART_TRANS_KP * posErrY + CART_TRANS_KD * velErrY + CART_TRANS_KI * errSumY_;

    // // a control - need to be careful of angle error
    // float posErrA = angle_diff(cmd.position_.a_, cartPos_.a_);
    // float velErrA = cmd.velocity_.a_ - cartVel_.a_;
    // errSumA_ += posErrA * dt;
    // float a_cmd = cmd.velocity_.a_ + CART_ROT_KP * posErrA + CART_ROT_KD * velErrA + CART_ROT_KI * errSumA_;

    // if(trajRunning_)
    // {
    //     PLOGI.printf("CartesianControlX: [PosErr: %.4f, VelErr: %.4f, ErrSum: %.4f]\n", posErrX, velErrX, errSumX_);
    // }

    // TODO: Convert back to closed loop at some point
    float x_cmd = cmd.velocity_.vx_;
    float y_cmd = cmd.velocity_.vy_;
    float a_cmd = cmd.velocity_.va_;

    return {x_cmd, y_cmd, a_cmd};
}

bool RobotController::checkForCompletedTrajectory(const PVTPoint cmd)
{
    // TODO: Switch to config
    float trans_pos_err = TRANS_POS_ERR_COARSE;
    float ang_pos_err = ANG_POS_ERR_COARSE;
    float trans_vel_err = TRANS_VEL_ERR_COARSE;
    float ang_vel_err = ANG_VEL_ERR_COARSE;
    if(fineMode_)
    {
      trans_pos_err = TRANS_POS_ERR_FINE;
      ang_pos_err = ANG_POS_ERR_FINE;
      trans_vel_err = TRANS_VEL_ERR_FINE;
      ang_vel_err = ANG_VEL_ERR_FINE;
    }


    // TODO: Simplfy this and make it readable
    if(cmd.velocity_ == Velocity(0,0,0) &&
       fabs(cartVel_.vx_) < trans_vel_err && 
       fabs(cartVel_.vy_) < trans_vel_err && 
       fabs(cartVel_.va_) < ang_vel_err &&
       (velOnlyMode_ || 
       (fabs(goalPos_.x_ - cartPos_.x_) < trans_pos_err &&
        fabs(goalPos_.y_ - cartPos_.y_) < trans_pos_err &&
        fabs(angle_diff(goalPos_.a_, cartPos_.a_)) < ang_pos_err )) )
    {
        PLOGI.printf("Reached goal");
        return true;
    } 
    else
    {
        return false;
    }
}

void RobotController::enableAllMotors()
{
    // TODO: port this
    enabled_ = true;
    PLOGI.printf("Enabling motors");
}

void RobotController::disableAllMotors()
{
    // TODO: port this
    enabled_ = false;
    PLOGI.printf("Disabling motors");
}

void RobotController::inputPosition(float x, float y, float a)
{
    // TODO: Fix this or scrap it
    (void) x;
    (void) y;
    (void) a;
    
    if(fineMode_)
    {  
    //   // Retrieve new state estimate
    //   Eigen::Vector3f x_hat = kf_.state();
    //   cartPos_.x_ = x_hat(0,0);
    //   cartPos_.y_ = x_hat(1,0);
    //   cartPos_.a_ = x_hat(2,0);
    }
}

void RobotController::computeOdometry()
{  
 
    // TODO: Move message parsing to a different function
    std::string msg = "";
    float local_cart_vel[3];
    if (serial_to_motor_driver_->isConnected())
    {
         msg = serial_to_motor_driver_->rcv();
    }

    if (msg.empty())
    {
        return;
    }
    else
    {
        int prev_idx = 0;
        int j = 0;
        for(uint i = 0; i < msg.length(); i++)
        {
            if(msg[i] == ',')
            {
                local_cart_vel[j] = std::stof(msg.substr(prev_idx, i - prev_idx));
                j++;
                prev_idx = i+1;
            }

            if (i == msg.length()-1)
            {
                local_cart_vel[j] = std::stof(msg.substr(prev_idx, std::string::npos));
            }
        }
        if(j != 2)
        {
            PLOGW.printf("Decode failed");
            return;
        }
    }
    
    PLOGD_(MOTION_LOG_ID).printf("Decoded velocity: %.3f, %.3f, %.3f", local_cart_vel[0], local_cart_vel[1], local_cart_vel[2]);

    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    cartVel_.vx_ = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    cartVel_.vy_ = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    cartVel_.va_ = local_cart_vel[2];

    // Compute time since last odom update
    // std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    // float dt = std::chrono::duration_cast<fsec>(curTime - prevOdomLoopTime_).count()
    // prevOdomLoopTime_ = curTime;

    // Retrieve new state estimate
    // Eigen::Vector3f x_hat = kf_.state();
    // cartPos_.x_ = x_hat(0,0);
    // cartPos_.y_ = x_hat(1,0);
    // cartPos_.a_ = x_hat(2,0);

    // TODO: Integrate velocity to get position
}

void RobotController::setCartVelCommand(Velocity target_vel)
{
    if (trajRunning_) 
    {
        PLOGD_(MOTION_LOG_ID).printf("CartVelCmd: [vx: %.4f, vy: %.4f, va: %.4f]", target_vel.vx_, target_vel.vy_, target_vel.va_);
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
        PLOGD_(MOTION_LOG_ID).printf("Sending to motors: [%s]", s.c_str());
    }

    if (serial_to_motor_driver_->isConnected())
    {
        serial_to_motor_driver_->send(s);
    }
}
