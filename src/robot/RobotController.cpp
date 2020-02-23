#include "RobotController.h"
#include "globals.h"
#include "utils.h"
#include <math.h>
#include <LinearAlgebra.h>

const DynamicLimits FINE_LIMS = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE};
const DynamicLimits COARSE_LIMS = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE};

const float RAD_PER_SEC_TO_STEPS_PER_SEC = STEPPER_PULSE_PER_REV/  (2.0 * PI);
const float SECONDS_TO_MICROSECONDS = 1000000;

RobotController::RobotController(HardwareSerial& debug, StatusUpdater& statusUpdater)
: prevPositionUpdateTime_(millis()),
  prevControlLoopTime_(millis()),
  prevUpdateLoopTime_(millis()),
  prevOdomLoopTime_(millis()),
  debug_(debug),
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
  predict_once(false),
  statusUpdater_(statusUpdater),
  motor_velocities({0,0,0,0})
{
    // Setup motors
    StepperDriver.init();
    motors[0] = StepperDriver.newAxis(PIN_PULSE_0, PIN_DIR_0, 255, STEPPER_PULSE_PER_REV);
    motors[1] = StepperDriver.newAxis(PIN_PULSE_1, PIN_DIR_1, 255, STEPPER_PULSE_PER_REV);
    motors[2] = StepperDriver.newAxis(PIN_PULSE_2, PIN_DIR_2, 255, STEPPER_PULSE_PER_REV);
    motors[3] = StepperDriver.newAxis(PIN_PULSE_3, PIN_DIR_3, 255, STEPPER_PULSE_PER_REV);

    // Setup enable pin
    digitalWrite(PIN_ENABLE_ALL, HIGH);
    pinMode(PIN_ENABLE_ALL, OUTPUT);

    // Setup Kalman filter
    double dt = 0.1;
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
    trajRunning_ = true;
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;
    fineMode_ = false;
    enableAllMotors();
    #ifdef PRINT_DEBUG
    debug_.println("Starting move");
    #endif
}

void RobotController::moveToPositionRelative(float x, float y, float a)
{
    goalPos_ = Point(cartPos_.x_ + x, cartPos_.y_ + y, cartPos_.a_ + a);
    trajGen_.generate(cartPos_, goalPos_, COARSE_LIMS);
    trajRunning_ = true;
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;
    fineMode_ = false;
    enableAllMotors();
    #ifdef PRINT_DEBUG
    debug_.println("Starting move");
    #endif
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    goalPos_ = Point(x,y,a);
    trajGen_.generate(cartPos_, goalPos_, FINE_LIMS);
    trajRunning_ = true;
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;
    fineMode_ = false;
    enableAllMotors();
    #ifdef PRINT_DEBUG
    debug_.println("Starting move");
    #endif
}

void RobotController::update()
{
    // Compute the amount of time since the last update
    unsigned long curMillis = millis();
    unsigned long dt_ms = curMillis - prevUpdateLoopTime_;
    float dt = static_cast<float>((dt_ms) / 1000.0); // Convert to seconds
    controller_time_averager_.input(static_cast<float>(curMillis - prevUpdateLoopTime_));
    prevUpdateLoopTime_ = curMillis;  
    
    PVTPoint cmd;
    if(trajRunning_)
    {
        float dt = static_cast<float>((curMillis - trajStartTime_) / 1000.0); // Convert to seconds
        cmd = trajGen_.lookup(dt);

        #ifdef PRINT_DEBUG
        debug_.println("");
        debug_.print("Target: ");
        cmd.print(debug_);
        debug_.println("");
        debug_.print("Est Vel: ");
        cartVel_.print(debug_);
        debug_.println("");
        debug_.print("Est Pos: ");
        cartPos_.print(debug_);
        debug_.println("");
        #endif
       
        // Stop trajectory
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
        // Force velocity to 0
        cartVel_.x_ = 0;
        cartVel_.y_ = 0;
        cartVel_.a_ = 0;
      
        // Force cmd to 0
        cmd.position_.x_ = cartPos_.x_;
        cmd.position_.y_ = cartPos_.y_;
        cmd.position_.a_ = cartPos_.a_;;
        cmd.velocity_.x_ = 0;
        cmd.velocity_.y_ = 0;
        cmd.velocity_.a_ = 0;
        cmd.time_ = 0; // Doesn't matter, not used

        // Make sure integral terms in controller don't wind up
        errSumX_ = 0;
        errSumY_ = 0;
        errSumA_ = 0;
    }

    // Run controller and odometry update
    computeControl(cmd);
    computeOdometry();

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x_, cartPos_.y_, cartPos_.a_);
    statusUpdater_.updateVelocity(cartVel_.x_, cartVel_.y_, cartVel_.a_);
    statusUpdater_.updateLoopTimes(static_cast<int>(controller_time_averager_.mean()), static_cast<int>(position_time_averager_.mean()));
    mat P = kf_.cov();
    statusUpdater_.updatePositionConfidence(P(0,0), P(1,1), P(2,2));
    statusUpdater_.updateInProgress(trajRunning_);
}


void RobotController::computeControl(PVTPoint cmd)
{
    
    // Recompute the loop update time to get better accuracy
    unsigned long curMillis = millis();
    float dt = static_cast<float>((curMillis - prevControlLoopTime_) / 1000.0); // Convert to seconds
    prevControlLoopTime_ = curMillis;


    // TODO: Make controller use matrix math

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

    setCartVelCommand(x_cmd, y_cmd, a_cmd);
}

bool RobotController::checkForCompletedTrajectory(PVTPoint cmd)
{
    // TODO split out vel error checks into trans/angle too, and update cmd vel to use this
    float eps = 0.01;

    float trans_pos_err = TRANS_POS_ERR_COARSE;
    float ang_pos_err = ANG_POS_ERR_COARSE;
    if(fineMode_)
    {
      trans_pos_err = TRANS_POS_ERR_FINE;
      ang_pos_err = ANG_POS_ERR_FINE;
    }
    if(cmd.velocity_.x_ == 0 && cmd.velocity_.y_ == 0 && cmd.velocity_.a_ == 0 &&
       fabs(cartVel_.x_) < eps && fabs(cartVel_.y_) < eps && fabs(cartVel_.a_) < eps &&
       fabs(goalPos_.x_ - cartPos_.x_) < trans_pos_err &&
       fabs(goalPos_.y_ - cartPos_.y_) < trans_pos_err &&
       fabs(angle_diff(goalPos_.a_, cartPos_.a_)) < ang_pos_err )
    {
        #ifdef PRINT_DEBUG
        debug_.println("Reached goal");
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
    digitalWrite(PIN_ENABLE_ALL, LOW);
    for(int i = 0; i < 4; i++)
    {
        StepperDriver.enable(motors[i]);
    }
    enabled_ = true;
    #ifdef PRINT_DEBUG
    debug_.println("Enabling motors");
    #endif
}

void RobotController::disableAllMotors()
{
    digitalWrite(PIN_ENABLE_ALL, HIGH);
    for(int i = 0; i < 4; i++)
    {
        StepperDriver.disable(motors[i]);
    }
    enabled_ = false;
    #ifdef PRINT_DEBUG
    debug_.println("Disabling motors");
    #endif
}

void RobotController::inputPosition(float x, float y, float a)
{
    if(fineMode_)
    {
      // Updat kalman filter for position observation
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
  
      // Compute update rate
      unsigned long curMillis = millis();
      unsigned long dt = curMillis - prevPositionUpdateTime_;
      prevPositionUpdateTime_ = curMillis;
      position_time_averager_.input(dt);
    }
}

void RobotController::computeOdometry()
{  
    // Do forward kinematics to compute local cartesian velocity
    float local_cart_vel[3];
    float s0 = 0.5 * WHEEL_DIAMETER * sin(PI/4.0);
    float c0 = 0.5 * WHEEL_DIAMETER * cos(PI/4.0);
    float d0 = WHEEL_DIAMETER / (4.0 * WHEEL_DIST_FROM_CENTER);
    local_cart_vel[0] = -c0 * motor_velocities[0] + s0 * motor_velocities[1] + c0 * motor_velocities[2] - s0 * motor_velocities[3];
    local_cart_vel[1] =  s0 * motor_velocities[0] + c0 * motor_velocities[1] - s0 * motor_velocities[2] - c0 * motor_velocities[3];
    local_cart_vel[2] =  d0 * motor_velocities[0] + d0 * motor_velocities[1] + d0 * motor_velocities[2] + d0 * motor_velocities[3];

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
//    debug_.print("CartVelCmd: [vx: ");
//    debug_.print(vx, 4);
//    debug_.print(", vy ");
//    debug_.print(vy, 4);
//    debug_.print(", va: ");
//    debug_.print(va, 4);
//    debug_.println("]");

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
//        debug_.println("Capping vx velocity");
        vx = sgn(vx) * max_trans_speed;
    }
    if(fabs(vy) > max_trans_speed)
    {
//        debug_.println("Capping vy velocity");
        vy = sgn(vy) * max_trans_speed;
    }
    if(fabs(va) > max_rot_speed)
    {
//        debug_.println("Capping angular velocity");
        va = sgn(va) * max_rot_speed;
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] =  cA * vx + sA * vy;
    local_cart_vel[1] = -sA * vx + cA * vy;
    local_cart_vel[2] = va;

    // Convert local velocities to wheel speeds with inverse kinematics
    float s0 = sin(PI/4);
    float c0 = cos(PI/4);
    motor_velocities[0] = 1/WHEEL_DIAMETER * (-c0*local_cart_vel[0] + s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[1] = 1/WHEEL_DIAMETER * ( s0*local_cart_vel[0] + c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[2] = 1/WHEEL_DIAMETER * ( c0*local_cart_vel[0] - s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[3] = 1/WHEEL_DIAMETER * (-s0*local_cart_vel[0] - c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);

    // Set the commanded values for each motor
    for (int i = 0; i < 4; i++)
    {
        float vel = motor_velocities[i];
        uint16_t delay_us = 0; // This works for if vel is 0

        // Compute motor direction
        if(vel > 0)
        {
            StepperDriver.setDir(motors[i], FORWARD);
        }
        else
        {
            vel = -vel;
            StepperDriver.setDir(motors[i], BACKWARD); 
        }

        // Compute delay between steps to achieve desired velocity
        if(vel != 0)
        {
            delay_us = static_cast<uint16_t>(SECONDS_TO_MICROSECONDS /(vel * RAD_PER_SEC_TO_STEPS_PER_SEC));
        }

        // Update motor
        StepperDriver.setDelay(motors[i], delay_us);
    }
}
