#include "RobotController.h"
#include "globals.h"
#include <math.h>
#include <LinearAlgebra.h>

#define PROCESS_NOISE_SCALE 0.05;
#define MEAS_NOISE_SCALE 0.01;

RobotController::RobotController(HardwareSerial& debug, StatusUpdater& statusUpdater)
: motors{
    Motor(PIN_PWM_1, PIN_DIR_1, PIN_ENCA_1, PIN_ENCB_1, MOTOR_KP, MOTOR_KI, MOTOR_KD),
    Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENCA_2, PIN_ENCB_2, MOTOR_KP, MOTOR_KI, MOTOR_KD),
    Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENCA_3, PIN_ENCB_3, MOTOR_KP, MOTOR_KI, MOTOR_KD),
    Motor(PIN_PWM_4, PIN_DIR_4, PIN_ENCA_4, PIN_ENCB_4, MOTOR_KP, MOTOR_KI, MOTOR_KD)},
  prevPositionUpdateTime_(millis()),
  prevControlLoopTime_(millis()),
  prevUpdateLoopTime_(millis()),
  prevOdomLoopTime_(millis()),
  debug_(debug),
  enabled_(false),
  trajGen_(debug),
  cartPos_(0),
  cartVel_(0),
  trajRunning_(false),
  trajStartTime_(0),
  errSumX_(0),
  errSumY_(0),
  errSumA_(0),
  statusUpdater_(statusUpdater)
{
    pinMode(PIN_ENABLE,OUTPUT);

    // Setup Kalman filter
    double dt = 0.1;
    mat A = mat::identity(6); // Doesn't matter right now since we update this at each time step
    mat B = mat::zeros(6,3);
    mat C = mat::zeros(6,3);
    mat Q = mat::identity(6) * PROCESS_NOISE_SCALE;
    mat R = mat::identity(3) * MEAS_NOISE_SCALE;
    mat P = mat::identity(6);
    B(3,0) = 1;
    B(4,1) = 1;
    B(5,2) = 1;
    C(0,0) = 1;
    C(1,1) = 1;
    C(2,2) = 1;
    kf_ = KalmanFilter(dt, A, B, C, Q, R, P);
    kf_.init();
}

void RobotController::moveToPosition(float x, float y, float a)
{
    trajGen_.generate(cartPos_, Point(x,y,a));
    trajRunning_ = true;
    trajStartTime_ = millis();
    prevControlLoopTime_ = 0;
    enableAllMotors();
    debug_.println("Starting move");
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    // TODO
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

// Print covariance matrix
//        String s;
//        mat P = kf_.cov();
//        P.print(s);
//        debug_.println(s);
       
        // Stop trajectory
        if (checkForCompletedTrajectory(cmd))
        {
            disableAllMotors();
            trajRunning_ = false;
        }
    }
    else
    {
        // Reset Kalman Filter and make sure that velocity is forced to 0
        mat x_hat = kf_.state();
        x_hat(3,0) = 0;
        x_hat(4,0) = 0;
        x_hat(5,0) = 0;
        kf_.init(0, x_hat);
        
        // Force cmd to 0
        cmd.position_.x_ = cartPos_.x_;
        cmd.position_.y_ = cartPos_.y_;
        cmd.position_.a_ = cartPos_.a_;;
        cmd.velocity_.x_ = 0;
        cmd.velocity_.y_ = 0;
        cmd.velocity_.a_ = 0;
        cmd.time_ = 0; // Doesn't matter, not used

        // Make sure integral terms in controller  don't wind up
        errSumX_ = 0;
        errSumY_ = 0;
        errSumA_ = 0;
    }

    // Run controller and motor update
    computeControl(cmd);
    updateMotors();
    computeOdometry();

    // Update status 
    statusUpdater_.updatePosition(cartPos_.x_, cartPos_.y_, cartPos_.a_);
    statusUpdater_.updateVelocity(cartVel_.x_, cartVel_.y_, cartVel_.a_);
    statusUpdater_.updateLoopTimes(static_cast<int>(controller_time_averager_.mean()), static_cast<int>(position_time_averager_.mean()));
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

    // a control 
    float posErrA = cmd.position_.a_ - cartPos_.a_;
    float velErrA = cmd.velocity_.a_ - cartVel_.a_;
    errSumA_ += posErrA * dt;
    float a_cmd = cmd.velocity_.a_ + CART_ROT_KP * posErrA + CART_ROT_KD * velErrA + CART_ROT_KI * errSumA_;

    setCartVelCommand(x_cmd, y_cmd, a_cmd);
}

bool RobotController::checkForCompletedTrajectory(PVTPoint cmd)
{
    float eps = 0.01;
    if(cmd.velocity_.x_ == 0 && cmd.velocity_.y_ == 0 && cmd.velocity_.a_ == 0 &&
       fabs(cartVel_.x_) < eps && fabs(cartVel_.y_) < eps && fabs(cartVel_.a_) < eps)
    {
        return true;
    } 
    else
    {
        return false;
    }
}

void RobotController::enableAllMotors()
{
    digitalWrite(PIN_ENABLE, 1);
    enabled_ = true;
    debug_.println("Enabling motors");
}

void RobotController::disableAllMotors()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].setCommand(0);
    }
    digitalWrite(PIN_ENABLE, 0);
    enabled_ = false;
    debug_.println("Disabling motors");
}

void RobotController::inputPosition(float x, float y, float a)
{
    // Updat kalman filter for position observation
    mat z = mat::zeros(6,1);
    z(0,0) = x;
    z(1,0) = y;
    z(2,0) = a;
    kf_.update(z);

    // Retrieve new state estimate
    mat x_hat = kf_.state();
    cartPos_.x_ = x_hat(0,0);
    cartPos_.y_ = x_hat(1,0);
    cartPos_.a_ = x_hat(2,0);
    cartVel_.x_ = x_hat(3,0);
    cartVel_.y_ = x_hat(4,0);
    cartVel_.a_ = x_hat(5,0);

    // Compute update rate
    unsigned long curMillis = millis();
    unsigned long dt = curMillis - prevPositionUpdateTime_;
    prevPositionUpdateTime_ = curMillis;
    position_time_averager_.input(dt);

}

void RobotController::updateMotors()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].runLoop();
    }
}

void RobotController::computeOdometry()
{
    // Get wheel velocities from each motor
    float motor_vel[4];
    for(int i = 0; i < 4; i++)
    {
        // Need to convert from revs/sec to rad/sec
        motor_vel[i] = FUDGE_FACTOR * 2.0 * PI * motors[i].getCurrentVelocity();
    }

    // Do forward kinematics to compute local cartesian velocity
    float local_cart_vel[3];
    float s0 = 0.5 * WHEEL_DIAMETER * sin(PI/4.0);
    float c0 = 0.5 * WHEEL_DIAMETER * cos(PI/4.0);
    float d0 = WHEEL_DIAMETER / (4.0 * WHEEL_DIST_FROM_CENTER);
    local_cart_vel[0] = -c0 * motor_vel[0] + s0 * motor_vel[1] + c0 * motor_vel[2] - s0 * motor_vel[3];
    local_cart_vel[1] =  s0 * motor_vel[0] + c0 * motor_vel[1] - s0 * motor_vel[2] - c0 * motor_vel[3];
    local_cart_vel[2] =  d0 * motor_vel[0] + d0 * motor_vel[1] + d0 * motor_vel[2] + d0 * motor_vel[3];

    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float global_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    global_cart_vel[0] = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    global_cart_vel[1] = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    global_cart_vel[2] = local_cart_vel[2];

    // Compute time since last odom update
    unsigned long curMillis = millis();
    float dt = static_cast<float>((curMillis - prevOdomLoopTime_) / 1000.0); // Convert to seconds
    prevOdomLoopTime_ = curMillis;    

    // Kalman filter prediction using the velocity measurment from the previous step
    // to predict our current position
    mat u = mat::zeros(3,1);
    u(0,0) = global_cart_vel[0];
    u(1,0) = global_cart_vel[1];
    u(2,0) = global_cart_vel[2];
    mat A = mat::identity(6);
    A(0,3) = dt;
    A(1,4) = dt;
    A(2,5) = dt;
    A(3,3) = 0;
    A(4,4) = 0;
    A(5,5) = 0;
    kf_.predict(dt, A, u);

    // Retrieve new state estimate
    mat x_hat = kf_.state();
    cartPos_.x_ = x_hat(0,0);
    cartPos_.y_ = x_hat(1,0);
    cartPos_.a_ = x_hat(2,0);
    cartVel_.x_ = x_hat(3,0);
    cartVel_.y_ = x_hat(4,0);
    cartVel_.a_ = x_hat(5,0);
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
    
    // Note that this doesn't handle total translational magnitude correctly, that
    // is fine for what we are doing now.
    if(fabs(vx) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vx velocity");
        vx = sgn(vx) * MAX_TRANS_SPEED;
    }
    if(fabs(vy) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vy velocity");
        vy = sgn(vy) * MAX_TRANS_SPEED;
    }
    if(fabs(va) > MAX_ROT_SPEED)
    {
        debug_.println("Capping angular velocity");
        va = sgn(va) * MAX_ROT_SPEED;
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] =  cA * vx + sA * vy;
    local_cart_vel[1] = -sA * vx + cA * vy;
    local_cart_vel[2] = va;

    // Convert local velocities to wheel speeds with inverse kinematics
    float motorSpeed[4];
    float s0 = sin(PI/4);
    float c0 = cos(PI/4);
    motorSpeed[0] = 1/WHEEL_DIAMETER * (-c0*local_cart_vel[0] + s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motorSpeed[1] = 1/WHEEL_DIAMETER * ( s0*local_cart_vel[0] + c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motorSpeed[2] = 1/WHEEL_DIAMETER * ( c0*local_cart_vel[0] - s0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);
    motorSpeed[3] = 1/WHEEL_DIAMETER * (-s0*local_cart_vel[0] - c0*local_cart_vel[1] + WHEEL_DIST_FROM_CENTER*local_cart_vel[2]);

    // Set the commanded values for each motor
    for (int i = 0; i < 4; i++)
    {
        // Need to convert motor speed into revs/sec from rad/sec
        motorSpeed[i] = motorSpeed[i] / (2.0 * PI * FUDGE_FACTOR);
        motors[i].setCommand(motorSpeed[i]);
    }
}
