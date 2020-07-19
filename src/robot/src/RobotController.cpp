#include "RobotController.h"
#include "constants.h"
#include "utils.h"
#include <math.h>

const DynamicLimits FINE_LIMS = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE};
const DynamicLimits COARSE_LIMS = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE};

RobotController::RobotController(StatusUpdater& statusUpdater)
: prevPositionUpdateTime_(millis()),
  prevControlLoopTime_(millis()),
  prevUpdateLoopTime_(millis()),
  prevOdomLoopTime_(millis()),
  enabled_(false),
  trajGen_(debug),
  cartPos_(0),
  cartVel_(0),
  goalPos_(0),
  trajRunning_(false),
  trajStartTime_(0),
  errSumX_(0),
  errSumY_(0),
  errSumA_(0),
  fineMode_(true),
  velOnlyMode_(false),
  predict_once(false),
  statusUpdater_(statusUpdater),
  logger_(spdlog::get("robot_logger"))
{
    setCoarseMotorGains();
}

void RobotController::begin()
{
    // Setup Kalman filter
    double dt = 0.1;

    // TODO: New linear algerba library
    mat A = mat::identity(3); 
    mat B = mat::identity(3); // Doesn't matter right now since we update this at each time step
    mat C = mat::identity(3);
    mat Q = mat::identity(3) * PROCESS_NOISE_SCALE;
    mat R = mat::identity(3) * MEAS_NOISE_SCALE;
    mat P = mat::identity(3);
    kf_ = KalmanFilter(dt, A, B, C, Q, R, P);
    kf_.init();
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
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;

    if(fineMode_)
    {
        setFineMotorGains();
    }
    else
    {
        setCoarseMotorGains();
    }
    

    enableAllMotors();
    #ifdef PRINT_DEBUG
    logger_->info("Starting move");
    #endif
}

void RobotController::estop()
{
    #ifdef PRINT_DEBUG
    logger_->info("Estopping robot control");
    #endif 
    trajRunning_ = false;
    fineMode_ = true;
    velOnlyMode_ = false;
    disableAllMotors();
}

void RobotController::setCoarseMotorGains()
{
    motors_[MOTOR_IDX_FL].setGains(FRONT_MOTOR_KP_COARSE, FRONT_MOTOR_KI_COARSE, FRONT_MOTOR_KD_COARSE);
    motors_[MOTOR_IDX_FR].setGains(FRONT_MOTOR_KP_COARSE, FRONT_MOTOR_KI_COARSE, FRONT_MOTOR_KD_COARSE);
    motors_[MOTOR_IDX_BL].setGains(REAR_MOTOR_KP_COARSE, REAR_MOTOR_KI_COARSE, REAR_MOTOR_KD_COARSE);
    motors_[MOTOR_IDX_BR].setGains(REAR_MOTOR_KP_COARSE, REAR_MOTOR_KI_COARSE, REAR_MOTOR_KD_COARSE);
}

void RobotController::setFineMotorGains()
{
    motors_[MOTOR_IDX_FL].setGains(FRONT_MOTOR_KP_FINE, FRONT_MOTOR_KI_FINE, FRONT_MOTOR_KD_FINE);
    motors_[MOTOR_IDX_FR].setGains(FRONT_MOTOR_KP_FINE, FRONT_MOTOR_KI_FINE, FRONT_MOTOR_KD_FINE);
    motors_[MOTOR_IDX_BL].setGains(REAR_MOTOR_KP_FINE, REAR_MOTOR_KI_FINE, REAR_MOTOR_KD_FINE);
    motors_[MOTOR_IDX_BR].setGains(REAR_MOTOR_KP_FINE, REAR_MOTOR_KI_FINE, REAR_MOTOR_KD_FINE);
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

    #ifdef PRINT_DEBUG
    if (trajRunning_)
    {
        logger_->info("Est Vel: ");
        cartVel_.print(logger_);
        logger_->info("");
        logger_->info("Est Pos: ");
        cartPos_.print(logger_);
        logger_->info("");
    }
    #endif

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x_, cartPos_.y_, cartPos_.a_);
    statusUpdater_.updateVelocity(cartVel_.x_, cartVel_.y_, cartVel_.a_);
    mat P = kf_.cov();
    statusUpdater_.updatePositionConfidence(P(0,0), P(1,1), P(2,2));
}


void RobotController::runTraj(PVTPoint* cmd)
{
    float dt = static_cast<float>((millis() - trajStartTime_) / 1000.0); // Convert to seconds
    *cmd = trajGen_.lookup(dt);

    #ifdef PRINT_DEBUG
    logger_->info("");
    logger_->info("Target: ");
    cmd->print(logger_);
    logger_->info("");
    #endif
    
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
    unsigned long curMillis = millis();
    float dt = static_cast<float>((curMillis - prevControlLoopTime_) / 1000.0); // Convert to seconds
    prevControlLoopTime_ = curMillis;

    // x control 
    float posErrX = cmd.position_.x_ - cartPos_.x_;
    float velErrX = cmd.velocity_.x_ - cartVel_.x_;
    errSumX_ += posErrX * dt;
    float x_cmd = cmd.velocity_.x_ + CART_TRANS_KP * posErrX + CART_TRANS_KD * velErrX + CART_TRANS_KI * errSumX_;

    // y control 
    float posErrY = cmd.position_.y_ - cartPos_.y_;
    float velErrY = cmd.velocity_.y_ - cartVel_.y_;
    errSumY_ += posErrY * dt;
    float y_cmd = cmd.velocity_.y_ + CART_TRANS_KP * posErrY + CART_TRANS_KD * velErrY + CART_TRANS_KI * errSumY_;

    // a control - need to be careful of angle error
    float posErrA = angle_diff(cmd.position_.a_, cartPos_.a_);
    float velErrA = cmd.velocity_.a_ - cartVel_.a_;
    errSumA_ += posErrA * dt;
    float a_cmd = cmd.velocity_.a_ + CART_ROT_KP * posErrA + CART_ROT_KD * velErrA + CART_ROT_KI * errSumA_;

    #ifdef PRINT_DEBUG
    if(trajRunning_)
    {
        logger_->info("CartesianControlX: [PosErr:");
        logger_->info(sprintf("%.4f", posErrX));
        logger_->info(", VelErr:");
        logger_->info(sprintf("%.4f", velErrX));
        logger_->info(", ErrSum:");
        logger_->info(sprintf("%.4f", errSumX_));
        logger_->info("]");
    }
    #endif

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
        #ifdef PRINT_DEBUG
        logger_->info("Reached goal");
        #endif
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
    #ifdef PRINT_DEBUG
    logger_)->info("Enabling motors");
    #endif
}

void RobotController::disableAllMotors()
{
    // TODO: port this
    enabled_ = false;
    #ifdef PRINT_DEBUG
    spdlog::get("robot_logger")->info("Disabling motors");
    #endif
}

void RobotController::inputPosition(float x, float y, float a)
{
    if(fineMode_)
    {
      // Update kalman filter for position observation
      mat z = mat::zeros(3,1);
      z(0,0) = x;
      z(1,0) = y;
      z(2,0) = a;

      // Scale covariance estimate based on velocity due to measurment lag
      mat R = mat::zeros(3,3);
      R(0,0) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.x_);
      R(1,1) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.y_);
      R(2,2) = MEAS_NOISE_SCALE + MEAS_NOISE_VEL_SCALE_FACTOR * fabs(cartVel_.a_);
      kf_.update(z, R, debug_);
  
      // Retrieve new state estimate
      mat x_hat = kf_.state();
      cartPos_.x_ = x_hat(0,0);
      cartPos_.y_ = x_hat(1,0);
      cartPos_.a_ = x_hat(2,0);
    }
}

void RobotController::computeOdometry()
{  
 
    // TODO: Get from clearcore
    float local_cart_vel[3] = {0,0,0}

    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    cartVel_.x_ = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    cartVel_.y_ = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    cartVel_.a_ = local_cart_vel[2];

    // Compute time since last odom update
    unsigned long curMillis = millis();
    float dt = static_cast<float>((curMillis - prevOdomLoopTime_) / 1000.0); // Convert to seconds
    prevOdomLoopTime_ = curMillis;    

    // Kalman filter prediction using the velocity measurment from the previous step
    // to predict our current position, but only if we are moving
    if( !(cartVel_.x_ == 0 && cartVel_.y_ == 0 && cartVel_.a_ == 0) || !predict_once)
    {
      mat u = mat::zeros(3,1);
      u(0,0) = cartVel_.x_;
      u(1,0) = cartVel_.y_;
      u(2,0) = cartVel_.a_;
      mat B = mat::identity(3) * dt;
      kf_.predict(dt, B, u);
      predict_once = true;
    }

    // Retrieve new state estimate
    mat x_hat = kf_.state();
    cartPos_.x_ = x_hat(0,0);
    cartPos_.y_ = x_hat(1,0);
    cartPos_.a_ = x_hat(2,0);
}

void RobotController::setCartVelCommand(float vx, float vy, float va)
{
    #ifdef PRINT_DEBUG
    if (trajRunning_) 
    {
        logger_->info("CartVelCmd: [vx: ");
        logger_->info(sprintf("%.4f", vx));
        logger_->info(", vy: ");
        logger_->info(sprintf("%.4f", vy));
        logger_->info(", va: ");
        logger_->info(sprintf("%.4f", va));
        logger_->info("]");
    }
    #endif

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
//        spdlog::get("robot_logger")->info("Capping vx velocity");
        vx = sgn(vx) * max_trans_speed;
    }
    if(fabs(vy) > max_trans_speed)
    {
//        spdlog::get("robot_logger")->info("Capping vy velocity");
        vy = sgn(vy) * max_trans_speed;
    }
    if(fabs(va) > max_rot_speed)
    {
//        spdlog::get("robot_logger")->info("Capping angular velocity");
        va = sgn(va) * max_rot_speed;
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] =  cA * vx + sA * vy;
    local_cart_vel[1] = -sA * vx + cA * vy;
    local_cart_vel[2] = va;

    // TODO: Send to clearcore
}