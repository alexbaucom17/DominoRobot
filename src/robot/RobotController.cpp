#include "RobotController.h"
#include "constants.h"
#include "utils.h"
#include <math.h>
#include <LinearAlgebra.h>

const DynamicLimits FINE_LIMS = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE};
const DynamicLimits COARSE_LIMS = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE};

RobotController::RobotController(HardwareSerial& debug, StatusUpdater& statusUpdater)
: motors_{ Motor(PIN_PWM_0, PIN_DIR_0, PIN_ENC_A_0, PIN_ENC_B_0, MOTOR_KP, MOTOR_KI, MOTOR_KD),
           Motor(PIN_PWM_1, PIN_DIR_1, PIN_ENC_A_1, PIN_ENC_B_1, MOTOR_KP, MOTOR_KI, MOTOR_KD),
           Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENC_A_2, PIN_ENC_B_2, MOTOR_KP, MOTOR_KI, MOTOR_KD),
           Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENC_A_3, PIN_ENC_B_3, MOTOR_KP, MOTOR_KI, MOTOR_KD) },
  prevPositionUpdateTime_(millis()),
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
  statusUpdater_(statusUpdater)
{
}

void RobotController::begin()
{
    // Setup pins
    digitalWrite(PIN_ENABLE_0, LOW);
    digitalWrite(PIN_ENABLE_1, LOW);
    digitalWrite(PIN_ENABLE_2, LOW);
    digitalWrite(PIN_ENABLE_3, LOW);
    pinMode(PIN_ENABLE_0, OUTPUT);
    pinMode(PIN_ENABLE_1, OUTPUT);
    pinMode(PIN_ENABLE_2, OUTPUT);
    pinMode(PIN_ENABLE_3, OUTPUT);

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
    fineMode_ = false;
    startTraj();
}

void RobotController::moveToPositionRelative(float x, float y, float a)
{
    goalPos_ = Point(cartPos_.x_ + x, cartPos_.y_ + y, cartPos_.a_ + a);
    trajGen_.generate(cartPos_, goalPos_, COARSE_LIMS);
    fineMode_ = false;
    startTraj();
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    goalPos_ = Point(x,y,a);
    trajGen_.generate(cartPos_, goalPos_, FINE_LIMS);
    fineMode_ = false;
    startTraj();

}

void RobotController::startTraj()
{
    trajRunning_ = true;
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;
    enableAllMotors();
    #ifdef PRINT_DEBUG
    debug_.println("Starting move");
    #endif
}

void RobotController::estop()
{
    #ifdef PRINT_DEBUG
    debug_.println("Estopping robot control");
    #endif 
    trajRunning_ = false;
    fineMode_ = true;
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

    // Update the motors
    for (int i = 0; i < 4; i++)
    {
        motors_[i].runLoop();
    }

    #ifdef PRINT_DEBUG
    if (trajRunning_)
    {
        debug_.print("Est Vel: ");
        cartVel_.print(debug_);
        debug_.println("");
        debug_.print("Est Pos: ");
        cartPos_.print(debug_);
        debug_.println("");
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
    debug_.println("");
    debug_.print("Target: ");
    cmd->print(debug_);
    debug_.println("");
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
    digitalWrite(PIN_ENABLE_0, HIGH);
    digitalWrite(PIN_ENABLE_1, HIGH);
    digitalWrite(PIN_ENABLE_2, HIGH);
    digitalWrite(PIN_ENABLE_3, HIGH);
    enabled_ = true;
    #ifdef PRINT_DEBUG
    debug_.println("Enabling motors");
    #endif
}

void RobotController::disableAllMotors()
{
    digitalWrite(PIN_ENABLE_0, LOW);
    digitalWrite(PIN_ENABLE_1, LOW);
    digitalWrite(PIN_ENABLE_2, LOW);
    digitalWrite(PIN_ENABLE_3, LOW);
    enabled_ = false;
    #ifdef PRINT_DEBUG
    debug_.println("Disabling motors");
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
    // Read velocities from the motors
    float motor_velocities[4];
    for (int i = 0; i < 4; i++)
    {
        motor_velocities[i] = motors_[i].getCurrentVelocity();
    }

    #ifdef PRINT_DEBUG
    if (trajRunning_)
    {
        debug_.print("MotorMeasured: [");
        debug_.print(motor_velocities[0], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[1], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[2], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[3], 4);
        debug_.println("]");
    }
    #endif

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
    #ifdef PRINT_DEBUG
    if (trajRunning_) 
    {
        debug_.print("CartVelCmd: [vx: ");
        debug_.print(vx, 4);
        debug_.print(", vy ");
        debug_.print(vy, 4);
        debug_.print(", va: ");
        debug_.print(va, 4);
        debug_.println("]");
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
    float motor_velocities[4];
    float s0 = sin(PI/4);
    float c0 = cos(PI/4);
    motor_velocities[0] = 1/WHEEL_DIAMETER * (-c0*local_cart_vel[0] + s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[1] = 1/WHEEL_DIAMETER * ( s0*local_cart_vel[0] + c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[2] = 1/WHEEL_DIAMETER * ( c0*local_cart_vel[0] - s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motor_velocities[3] = 1/WHEEL_DIAMETER * (-s0*local_cart_vel[0] - c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);

    #ifdef PRINT_DEBUG
    if (trajRunning_)
    {
        debug_.print("MotorCommands: [");
        debug_.print(motor_velocities[0], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[1], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[2], 4);
        debug_.print(", ");
        debug_.print(motor_velocities[3], 4);
        debug_.println("]");
    }
    #endif

    // Send the commanded velocity for each motor
    for (int i = 0; i < 4; i++)
    {
        motors_[i].setCommand(motor_velocities[i]);
    }

}
