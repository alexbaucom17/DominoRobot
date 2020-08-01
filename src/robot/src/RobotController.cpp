#include "RobotController.h"

#include <math.h>
// #include <Eigen/Core>
#include <plog/Log.h>

#include "constants.h"
#include "utils.h"


const DynamicLimits FINE_LIMS = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE};
const DynamicLimits COARSE_LIMS = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE};

typedef std::chrono::duration<float> fsec;

RobotController::RobotController(StatusUpdater& statusUpdater)
: prevControlLoopTime_(std::chrono::steady_clock::now()),
  prevOdomLoopTime_(std::chrono::steady_clock::now()),
  trajStartTime_(std::chrono::steady_clock::now()),
  enabled_(false),
  trajGen_(),
  cartPos_(0),
  cartVel_(0),
  goalPos_(0),
  trajRunning_(false),
  errSumX_(0),
  errSumY_(0),
  errSumA_(0),
  fineMode_(true),
  velOnlyMode_(false),
  predict_once(false),
  statusUpdater_(statusUpdater),
  serial_to_motor_driver_(CLEARCORE_USB)
{
    // // Setup Kalman filter
    // Eigen::Matrix3f A = Eigen::Matrix3f::Identity(); 
    // Eigen::Matrix3f B = Eigen::Matrix3f::Identity(); // Doesn't matter right now since we update this at each time step
    // Eigen::Matrix3f C = Eigen::Matrix3f::Identity(); 
    // Eigen::Matrix3f Q = Eigen::Matrix3f::Identity()  * PROCESS_NOISE_SCALE;
    // Eigen::Matrix3f R = Eigen::Matrix3f::Identity()  * MEAS_NOISE_SCALE;
    // Eigen::Matrix3f P = Eigen::Matrix3f::Identity(); 
    // kf_ = KalmanFilter(A, B, C, Q, R, P);
    // kf_.init();

    // TODO: setup serial with factory for easier offline testing
    
}

void RobotController::moveToPosition(float x, float y, float a)
{
    goalPos_ = Point(x,y,a);
    trajGen_.generate(cartPos_, goalPos_, COARSE_LIMS);
    fineMode_ = false;
    velOnlyMode_ = false;
    startTraj();
}

void RobotController::moveToPositionRelative(float x, float y, float a)
{
    goalPos_ = Point(cartPos_.x_ + x, cartPos_.y_ + y, cartPos_.a_ + a);
    trajGen_.generate(cartPos_, goalPos_, COARSE_LIMS);
    fineMode_ = false;
    velOnlyMode_ = false;
    startTraj();
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    goalPos_ = Point(x,y,a);
    trajGen_.generate(cartPos_, goalPos_, FINE_LIMS);
    fineMode_ = false;
    velOnlyMode_ = false;
    startTraj();

}

void RobotController::moveConstVel(float vx , float vy, float va, float t)
{
    trajGen_.generateConstVel(cartPos_, vx, vy, va, t, COARSE_LIMS);
    fineMode_ = false;
    velOnlyMode_ = true;
    startTraj();
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
        runTraj(&cmd);
    }
    else
    {       
        resetTraj(&cmd);
    }

    // Run controller and odometry update
    computeControl(cmd);
    computeOdometry();

    if (trajRunning_)
    {
        PLOGD_(MOTION_LOG_ID) << "Est Vel: " << cartVel_.toString();
        PLOGD_(MOTION_LOG_ID) << "Est Pos: " << cartPos_.toString();
    }

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x_, cartPos_.y_, cartPos_.a_);
    statusUpdater_.updateVelocity(cartVel_.x_, cartVel_.y_, cartVel_.a_);
    // Eigen::Matrix3f P = kf_.cov();
    // statusUpdater_.updatePositionConfidence(P(0,0), P(1,1), P(2,2));
}


void RobotController::runTraj(PVTPoint* cmd)
{
    std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    float dt = std::chrono::duration_cast<fsec>(curTime - trajStartTime_).count();
    *cmd = trajGen_.lookup(dt);

    PLOGD_(MOTION_LOG_ID) << "Target: " << cmd->toString();

    // Stop trajectory
    if (checkForCompletedTrajectory(*cmd))
    {
        disableAllMotors();
        trajRunning_ = false;
        // Re-enable fine mode at the end of a trajectory
        fineMode_ = true;
    }
}

void RobotController::resetTraj(PVTPoint* cmd)
{
    // Force velocity to 0
    cartVel_.x_ = 0;
    cartVel_.y_ = 0;
    cartVel_.a_ = 0;
    
    // Force cmd to 0
    cmd->position_.x_ = cartPos_.x_;
    cmd->position_.y_ = cartPos_.y_;
    cmd->position_.a_ = cartPos_.a_;;
    cmd->velocity_.x_ = 0;
    cmd->velocity_.y_ = 0;
    cmd->velocity_.a_ = 0;
    cmd->time_ = 0; // Doesn't matter, not used

    // Make sure integral terms in controller don't wind up
    errSumX_ = 0;
    errSumY_ = 0;
    errSumA_ = 0;

    // Flags
    fineMode_ = true;
    velOnlyMode_ = false;
}

void RobotController::computeControl(PVTPoint cmd)
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

    // TODO: Convert back to closed loop
    float x_cmd = cmd.velocity_.x_;
    float y_cmd = cmd.velocity_.y_;
    float a_cmd = cmd.velocity_.a_;

    setCartVelCommand(x_cmd, y_cmd, a_cmd);
}

bool RobotController::checkForCompletedTrajectory(const PVTPoint cmd)
{
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
    if(cmd.velocity_.x_ == 0 && cmd.velocity_.y_ == 0 && cmd.velocity_.a_ == 0 &&
       fabs(cartVel_.x_) < trans_vel_err && 
       fabs(cartVel_.y_) < trans_vel_err && 
       fabs(cartVel_.a_) < ang_vel_err &&
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
    if(fineMode_)
    {
      // Update kalman filter for position observation
    //   Eigen::Vector3f z = Eigen::Vector3f::Zero();
    //   z(0,0) = x;
    //   z(1,0) = y;
    //   z(2,0) = a;

    //   // Scale covariance estimate based on velocity due to measurment lag
    //   Eigen::Matrix3f R = Eigen::Matrix3f::Zero(); 
    //   R(0,0) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.x_);
    //   R(1,1) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.y_);
    //   R(2,2) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.a_);
    //   kf_.update(z, R);
  
    //   // Retrieve new state estimate
    //   Eigen::Vector3f x_hat = kf_.state();
    //   cartPos_.x_ = x_hat(0,0);
    //   cartPos_.y_ = x_hat(1,0);
    //   cartPos_.a_ = x_hat(2,0);
    }
}

void RobotController::computeOdometry()
{  
 
    std::string msg = "";
    float local_cart_vel[3];
    if (serial_to_motor_driver_.isConnected())
    {
         msg = serial_to_motor_driver_.rcv();
    }

    if (msg.empty())
    {
        PLOGW.printf("No message");
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
    cartVel_.x_ = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    cartVel_.y_ = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    cartVel_.a_ = local_cart_vel[2];

    // Compute time since last odom update
    // std::chrono::time_point<std::chrono::steady_clock> curTime = std::chrono::steady_clock::now();
    // float dt = std::chrono::duration_cast<fsec>(curTime - prevOdomLoopTime_).count()
    // prevOdomLoopTime_ = curTime;

    // // Kalman filter prediction using the velocity measurment from the previous step
    // // to predict our current position, but only if we are moving
    // if( !(cartVel_.x_ == 0 && cartVel_.y_ == 0 && cartVel_.a_ == 0) || !predict_once)
    // {
    //   Eigen::Vector3f u = Eigen::Vector3f::Zero();
    //   u(0,0) = cartVel_.x_;
    //   u(1,0) = cartVel_.y_;
    //   u(2,0) = cartVel_.a_;
    //   Eigen::Matrix3f B = Eigen::Matrix3f::Identity() * dt;
    //   kf_.predict(dt, B, u);
    //   predict_once = true;
    // }

    // Retrieve new state estimate
    // Eigen::Vector3f x_hat = kf_.state();
    // cartPos_.x_ = x_hat(0,0);
    // cartPos_.y_ = x_hat(1,0);
    // cartPos_.a_ = x_hat(2,0);

    // TODO: Integrate velocity to get position
}

void RobotController::setCartVelCommand(float vx, float vy, float va)
{
    if (trajRunning_) 
    {
        PLOGD_(MOTION_LOG_ID).printf("CartVelCmd: [vx: %.4f, vy: %.4f, va: %.4f]", vx, vy, va);
    }

    float max_trans_speed = COARSE_LIMS.max_trans_vel_;
    float max_rot_speed = COARSE_LIMS.max_rot_vel_;
    if(fineMode_)
    {
      max_trans_speed = FINE_LIMS.max_trans_vel_;
      max_rot_speed = FINE_LIMS.max_rot_vel_;
    }
    
    // Note that this doesn't handle total translational magnitude correctly, that
    // is fine for what we are doing now.
    if(fabs(vx) > max_trans_speed)
    {
        vx = sgn(vx) * max_trans_speed;
    }
    if(fabs(vy) > max_trans_speed)
    {
        vy = sgn(vy) * max_trans_speed;
    }
    if(fabs(va) > max_rot_speed)
    {
        va = sgn(va) * max_rot_speed;
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] =  cA * vx + sA * vy;
    local_cart_vel[1] = -sA * vx + cA * vy;
    local_cart_vel[2] = va;

    char buff[100];
    sprintf(buff, "%.4f,%.4f,%.4f",local_cart_vel[0], local_cart_vel[1], local_cart_vel[2]);
    std::string s = buff;

    if (local_cart_vel[0] != 0 || local_cart_vel[1] != 0 || local_cart_vel[2] != 0 )
    {
        PLOGD_(MOTION_LOG_ID).printf("Sending to motors: [%s]", s.c_str());
    }

    if (serial_to_motor_driver_.isConnected())
    {
        serial_to_motor_driver_.send(s);
    }
}
