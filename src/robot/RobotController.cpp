#include "RobotController.h"
#include "globals.h"
#include <math.h>

// Motor command loop time
#define targetDeltaMillis 15

// Motor control gains
const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

// Cartesian control gains
const float cartTransKp = 1;
const float cartTransKi = 0;
const float cartTransKd = 0;
const float cartRotKp = 1;
const float cartRotKi = 0;
const float cartRotKd = 0;

// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


RobotController::RobotController(HardwareSerial& debug)
: motors{
    Motor(PIN_PWM_1, PIN_DIR_1, PIN_ENCA_1, PIN_ENCB_1, Kp, Ki, Kd),
    Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENCA_2, PIN_ENCB_2, Kp, Ki, Kd),
    Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENCA_3, PIN_ENCB_3, Kp, Ki, Kd),
    Motor(PIN_PWM_4, PIN_DIR_4, PIN_ENCA_4, PIN_ENCB_4, Kp, Ki, Kd)},
  prevMotorLoopTime_(millis()),
  debug_(debug),
  enabled_(false),
  trajGen_(debug),
  cartPos_(0),
  cartVel_(0),
  trajRunning_(false),
  trajTime_(0),
  errSumX_(0),
  errSumY_(0),
  errSumA_(0),
  prevControlLoopTime_(0)
{
    pinMode(PIN_ENABLE,OUTPUT);
}

void RobotController::moveToPosition(float x, float y, float a)
{
    trajGen_.generate(cartPos_, Point(x,y,a));
    trajRunning_ = true;
    trajTime_ = millis();
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
    if(trajRunning_)
    {
        float dt = static_cast<float>((millis() - trajTime_) / 1000.0); // Convert to seconds
        trajTime_ = millis();
        debug_.print("dt: ");
        debug_.println(dt);
        PVTPoint cmd = trajGen_.lookup(dt);
        debug_.print("PVT: ");
        cmd.print(debug_);
        computeControl(cmd);
        updateMotors();

        // Stop trajectory
        if (checkForCompletedTrajectory(cmd))
        {
            disableAllMotors();
            trajRunning_ = false;
        }
    }
}

void RobotController::computeControl(PVTPoint cmd)
{
    float dt = cmd.time_ - prevControlLoopTime_;
    prevControlLoopTime_ = cmd.time_;
    
    // x control 
    float posErrX = cmd.position_.x_ - cartPos_.x_;
    float velErrX = cmd.velocity_.x_ - cartVel_.x_;
    errSumX_ += posErrX * dt;
    float x_cmd = cartTransKp * posErrX + cartTransKd * velErrX + cartTransKi * errSumX_;

    // y control 
    float posErrY = cmd.position_.y_ - cartPos_.y_;
    float velErrY = cmd.velocity_.y_ - cartVel_.y_;
    errSumY_ += posErrX * dt;
    float y_cmd = cartTransKp * posErrY + cartTransKd * velErrY + cartTransKi * errSumY_;

    // a control 
    float posErrA = cmd.position_.a_ - cartPos_.a_;
    float velErrA = cmd.velocity_.a_ - cartVel_.a_;
    errSumA_ += posErrA * dt;
    float a_cmd = cartRotKp * posErrA + cartRotKd * velErrA + cartRotKi * errSumA_;

    setCartVelCommand(x_cmd, y_cmd, a_cmd);
}

bool RobotController::checkForCompletedTrajectory(PVTPoint cmd)
{
    if(cmd.velocity_.x_ == 0 && cmd.velocity_.y_ == 0 && cmd.velocity_.a_ == 0)
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
    // TODO - fuse with odom
    // For now, just assume it is ground truth
    cartPos_.x_ = x;
    cartPos_.y_ = y;
    cartPos_.a_ = a;
}

void RobotController::updateMotors()
{
    if(enabled_)
    {
        unsigned long curTime = millis();
        unsigned long deltaMillis = curTime - prevMotorLoopTime_;
        if( deltaMillis > targetDeltaMillis)
        {
            for(int i = 0; i < 4; i++)
            {
                motors[i].runLoop();
            }
            computeOdometry(deltaMillis);
            prevMotorLoopTime_ = millis();
        }
    }
}

void RobotController::computeOdometry(unsigned long deltaMillis)
{
    // Get wheel velocities from each motor
    float motor_vel[4];
    for(int i = 0; i < 4; i++)
    {
        motor_vel[i] = motors[i].getCurrentVelocity();
    }

    // Do forward kinematics to compute local cartesian velocity
    float local_cart_vel[3];
    float s0 = 0.5 * WHEEL_DIAMETER * sin(PI/4);
    float c0 = 0.5 * WHEEL_DIAMETER * cos(PI/4);
    float d0 = WHEEL_DIAMETER / (4 * WHEEL_DIST_FROM_CENTER);
    local_cart_vel[0] = -c0 * motor_vel[0] + s0 * motor_vel[1] + c0 * motor_vel[2] - s0 * motor_vel[3];
    local_cart_vel[1] =  s0 * motor_vel[0] + c0 * motor_vel[1] - s0 * motor_vel[2] - c0 * motor_vel[3];
    local_cart_vel[2] =  d0 * motor_vel[0] + d0 * motor_vel[1] + d0 * motor_vel[2] + d0 * motor_vel[3];

    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    cartVel_.x_ = cA * local_cart_vel[0] - sA * local_cart_vel[1];
    cartVel_.y_ = sA * local_cart_vel[0] + cA * local_cart_vel[1];
    cartVel_.x_ = local_cart_vel[2];

    // Use global velocity to compute global position
    float dt = static_cast<float>(deltaMillis) / 1000.0;
    cartPos_.x_ += cartVel_.x_ * dt; 
    cartPos_.y_ += cartVel_.y_ * dt; 
    cartPos_.a_ += cartVel_.a_ * dt; 

}

void RobotController::setCartVelCommand(float vx, float vy, float va)
{
    // TODO - handle total transtlational vel magnitude correctly
    // Clamp input velocities
    if(abs(vx) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vx velocity");
        vx = sgn(vx) * MAX_TRANS_SPEED;
    }
    if(abs(vy) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vy velocity");
        vy = sgn(vy) * MAX_TRANS_SPEED;
    }
    if(abs(va) > MAX_ROT_SPEED)
    {
        debug_.println("Capping angular velocity");
        va = sgn(va) * MAX_ROT_SPEED;
    }

    // Convert input global velocities to local velocities
    float local_cart_vel[3];
    float cA = cos(cartPos_.a_);
    float sA = sin(cartPos_.a_);
    local_cart_vel[0] = cA * vx - sA * vy;
    local_cart_vel[1] = sA * vx + cA * vy;
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
        motors[i].setCommand(motorSpeed[i]);
    }
}
