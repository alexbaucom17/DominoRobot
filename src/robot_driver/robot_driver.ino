#include "ClearCore.h"
#include "SerialComms.h"
#include <math.h>

// From https://teknic-inc.github.io/ClearCore-library/ArduinoRef.html
// Serial = USB
// Serial0 + Serial1 = COM ports

// Globals
SerialComms comm(Serial0, Serial);

// Constants
#define WHEEL_RADIUS 0.0751
#define WHEEL_DIST_FROM_CENTER 0.4794
const float sq3 = sqrt(3.0);
const float rOver3 = WHEEL_RADIUS / 3.0;
const float radsPerSecondToStepsPerSecond = 6315 / (2 * PI);
// Motor mapping
#define MOTOR_FRONT_LEFT ConnectorM0 
#define MOTOR_FRONT_RIGHT ConnectorM1
#define MOTOR_REAR_CENTER ConnectorM2

struct CartVelocity
{
    float vx;
    float vy;
    float va;
};

struct MotorVelocity
{
    float v0;
    float v1;
    float v2;
};


CartVelocity decodeMsg(String msg)
{
    float vals[3];
    int prev_idx = 0;
    int j = 0;
    for(int i = 0; i < msg.length(); i++)
    {
      if(msg[i] == ',')
      {
        vals[j] = msg.substring(prev_idx, i).toFloat();
        j++;
        prev_idx = i+1;
      }

      if (i == msg.length())
      {
        vals[j] = msg.substring(prev_idx).toFloat();
      }
    }

    CartVelocity cv;
    cv.vx = vals[0];
    cv.vy = vals[1];
    cv.va = vals[2];
    return cv;
}

// https://www.wolframalpha.com/input/?i=%5B%5B-cosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5Bcosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5B0%2C-1%2Cd%5D%5D
MotorVelocity doIK(CartVelocity cmd)
{
    MotorVelocity motors;
    motors.v0 = 1/WHEEL_RADIUS * (-1 * sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va);
    motors.v1 = 1/WHEEL_RADIUS * (     sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va);
    motors.v2 = 1/WHEEL_RADIUS * (                       - 1   * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va);
    return motors;
}

void SendCommandsToMotors(MotorVelocity motors)
{
    MOTOR_FRONT_LEFT.MoveVelocity(motors.v0 * radsPerSecondToStepsPerSecond);
    MOTOR_FRONT_RIGHT.MoveVelocity(motors.v1 * radsPerSecondToStepsPerSecond);
    MOTOR_REAR_CENTER.MoveVelocity(motors.v2 * radsPerSecondToStepsPerSecond);
}

MotorVelocity ReadMotorSpeeds()
{
    MotorVelocity measured;
    measured.v0 = MOTOR_FRONT_LEFT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    measured.v1 = MOTOR_FRONT_RIGHT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    measured.v2 = MOTOR_REAR_CENTER.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    return measured;
}

// https://www.wolframalpha.com/input/?i=inv%28%5B%5B-cosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5Bcosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5B0%2C-1%2Cd%5D%5D%29
CartVelocity doFK(MotorVelocity motor_measured)
{
    CartVelocity robot_measured;
    robot_measured.vx = rOver3 * (-1 * sq3 * motor_measured.v0 + sq3 * motor_measured.v1);
    robot_measured.vy = rOver3 * ( motor_measured.v0 + motor_measured.v1 - 2.0 * motor_measured.v2); 
    robot_measured.va = rOver3 / WHEEL_DIST_FROM_CENTER * (motor_measured.v0 + motor_measured.v1 + motor_measured.v2);
    return robot_measured;

}

void ReportRobotVelocity(CartVelocity robot_v_measured)
{
    char msg[32];
    sprintf(msg, "%f,%f,%f",robot_v_measured.vx, robot_v_measured.vy, robot_v_measured.va);
    comm.send(msg);
}


void setup()
{
    Serial.begin(115200);
    
    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    // Enable motors
    MOTOR_FRONT_RIGHT.EnableRequest(true);
    MOTOR_FRONT_LEFT.EnableRequest(true);
    MOTOR_REAR_CENTER.EnableRequest(true);
}

void loop()
{
    String msg = comm.rcv();
    if(msg.length() != 0)
    {
        CartVelocity cmd_v = decodeMsg(msg);
        MotorVelocity motor_v = doIK(cmd_v);
        SendCommandsToMotors(motor_v);
        MotorVelocity motor_v_measured = ReadMotorSpeeds();
        CartVelocity robot_v_measured = doFK(motor_v_measured);
        ReportRobotVelocity(robot_v_measured);
    }

}
