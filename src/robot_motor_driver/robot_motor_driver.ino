#include "ClearCore.h"
#include "SerialComms.h"
#include <math.h>

// From https://teknic-inc.github.io/ClearCore-library/ArduinoRef.html
// Serial = USB
// Serial0 + Serial1 = COM ports

#define PRINT_DEBUG false

// Globals
SerialComms comm(Serial);

// Constants
#define WHEEL_RADIUS 0.0751
#define WHEEL_DIST_FROM_CENTER 0.4794
#define BELT_RATIO 4
#define STEPS_PER_REV 800
#define MOTOR_MAX_VEL_STEPS_PER_SECOND 10000
#define MOTOR_MAX_ACC_STEPS_PER_SECOND_SQUARED 100000
const float sq3 = sqrt(3.0);
const float rOver3 = WHEEL_RADIUS / 3.0;
const float radsPerSecondToStepsPerSecond = STEPS_PER_REV / (2 * PI);

// Motor mapping
#define MOTOR_FRONT_LEFT ConnectorM0 
#define MOTOR_FRONT_RIGHT ConnectorM1
#define MOTOR_REAR_CENTER ConnectorM2

float MOTOR_FRONT_LEFT_FAKE = 0;
float MOTOR_FRONT_RIGHT_FAKE = 0;
float MOTOR_REAR_CENTER_FAKE = 0;

struct CartVelocity
{
    float vx;
    float vy;
    float va;

    String toString() const
    {
        char s[100];
        sprintf(s, "[vx: %.4f, vy: %.4f, va: %.4f]", vx, vy, va);
        return static_cast<String>(s);
    }
};

struct MotorVelocity
{
    float v0;
    float v1;
    float v2;

    String toString() const
    {
        char s[100];
        sprintf(s, "[v0: %.4f, v1: %.4f, v2: %.4f]", v0, v1, v2);
        return static_cast<String>(s);
    }
};


CartVelocity decodeMsg(String msg)
{
#if PRINT_DEBUG
    comm.send("DEBUG Incoming message: " + msg);
#endif
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

      if (i == msg.length()-1)
      {
        vals[j] = msg.substring(prev_idx).toFloat();
      }
    }

    CartVelocity cv;
    cv.vx = vals[0];
    cv.vy = vals[1];
    cv.va = vals[2];
    
#if PRINT_DEBUG
    comm.send("DEBUG Decoded message: " + cv.toString());
#endif

    return cv;
}

// https://www.wolframalpha.com/input/?i=%5B%5B-cosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5Bcosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5B0%2C-1%2Cd%5D%5D
MotorVelocity doIK(CartVelocity cmd)
{
    MotorVelocity motors;
    motors.v0 = 1/WHEEL_RADIUS * (-1 * sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
    motors.v1 = 1/WHEEL_RADIUS * (     sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
    motors.v2 = 1/WHEEL_RADIUS * (                       - 1   * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
#if PRINT_DEBUG
    comm.send("DEBUG After IK: " + motors.toString());
#endif
    return motors;
}

void SendCommandsToMotors(MotorVelocity motors)
{
#if PRINT_DEBUG
    comm.send("DEBUG Send to motor: " + String(motors.v0 * radsPerSecondToStepsPerSecond));
#endif
    //MOTOR_FRONT_LEFT.MoveVelocity(motors.v0 * radsPerSecondToStepsPerSecond);
    //MOTOR_FRONT_RIGHT.MoveVelocity(motors.v1 * radsPerSecondToStepsPerSecond);
    //MOTOR_REAR_CENTER.MoveVelocity(motors.v2 * radsPerSecondToStepsPerSecond);
    MOTOR_FRONT_LEFT_FAKE  = motors.v0 * radsPerSecondToStepsPerSecond;
    MOTOR_FRONT_RIGHT_FAKE = motors.v1 * radsPerSecondToStepsPerSecond;
    MOTOR_REAR_CENTER_FAKE = motors.v2 * radsPerSecondToStepsPerSecond;
}

MotorVelocity ReadMotorSpeeds()
{
    MotorVelocity measured;
//    measured.v0 = MOTOR_FRONT_LEFT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
//    measured.v1 = MOTOR_FRONT_RIGHT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
//    measured.v2 = MOTOR_REAR_CENTER.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    measured.v0 = MOTOR_FRONT_LEFT_FAKE / radsPerSecondToStepsPerSecond;
    measured.v1 = MOTOR_FRONT_RIGHT_FAKE / radsPerSecondToStepsPerSecond;
    measured.v2 = MOTOR_REAR_CENTER_FAKE / radsPerSecondToStepsPerSecond;

#if PRINT_DEBUG
    comm.send("DEBUG Motor measured: " + measured.toString());
#endif

    return measured;
}

// https://www.wolframalpha.com/input/?i=inv%28%5B%5B-cosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5Bcosd%2830%29%2C+sind%2830%29%2C+d%5D%2C%5B0%2C-1%2Cd%5D%5D%29
CartVelocity doFK(MotorVelocity motor_measured)
{
    MotorVelocity wheel_speed;
    wheel_speed.v0 = motor_measured.v0 / float(BELT_RATIO);
    wheel_speed.v1 = motor_measured.v1 / float(BELT_RATIO);
    wheel_speed.v2 = motor_measured.v2 / float(BELT_RATIO);
    
    CartVelocity robot_measured;
    robot_measured.vx = rOver3 * (-1 * sq3 * wheel_speed.v0 + sq3 * wheel_speed.v1);
    robot_measured.vy = rOver3 * ( wheel_speed.v0 + wheel_speed.v1 - 2.0 * wheel_speed.v2); 
    robot_measured.va = rOver3 / WHEEL_DIST_FROM_CENTER * (wheel_speed.v0 + wheel_speed.v1 + wheel_speed.v2);
#if PRINT_DEBUG
    comm.send("DEBUG After FK: " + robot_measured.toString());
#endif
    return robot_measured;

}

void ReportRobotVelocity(CartVelocity robot_v_measured)
{
    char msg[32];
    sprintf(msg, "%.3f,%.3f,%.3f",robot_v_measured.vx, robot_v_measured.vy, robot_v_measured.va);
    comm.send(msg);
}


// Checks for any motor on/off requests before trying to parse for commanded speeds
bool handlePowerRequests(String msg)
{
    if(msg == "Power:ON")
    {
        MOTOR_FRONT_RIGHT.EnableRequest(true);
        MOTOR_REAR_CENTER.EnableRequest(true);
        MOTOR_FRONT_LEFT.EnableRequest(true);
#if PRINT_DEBUG
        comm.send("DEBUG Motors enabled");
#endif
        return true;
    }
    else if (msg == "Power:OFF")
    {
        MOTOR_FRONT_RIGHT.EnableRequest(false);
        MOTOR_REAR_CENTER.EnableRequest(false);
        MOTOR_FRONT_LEFT.EnableRequest(false);
#if PRINT_DEBUG
        comm.send("DEBUG Motors disabled");
#endif
        return true;
    }
    return false;
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
    MOTOR_FRONT_RIGHT.EnableRequest(false);
    MOTOR_FRONT_RIGHT.VelMax(MOTOR_MAX_VEL_STEPS_PER_SECOND);
    MOTOR_FRONT_RIGHT.AccelMax(MOTOR_MAX_ACC_STEPS_PER_SECOND_SQUARED);
    MOTOR_REAR_CENTER.EnableRequest(false);
    MOTOR_REAR_CENTER.VelMax(MOTOR_MAX_VEL_STEPS_PER_SECOND);
    MOTOR_REAR_CENTER.AccelMax(MOTOR_MAX_ACC_STEPS_PER_SECOND_SQUARED);
    MOTOR_FRONT_LEFT.EnableRequest(false);
    MOTOR_FRONT_LEFT.VelMax(MOTOR_MAX_VEL_STEPS_PER_SECOND);
    MOTOR_FRONT_LEFT.AccelMax(MOTOR_MAX_ACC_STEPS_PER_SECOND_SQUARED);

}

unsigned long prevLoopMillis = 0;
void loop()
{
    if (millis() - prevLoopMillis > 5000)
    {
      comm.send("DEBUG Motor driver connected");
      prevLoopMillis = millis();
    }
    
    String msg = comm.rcv();
    if(msg.length() != 0)
    {
        if (!handlePowerRequests(msg))
        {
            CartVelocity cmd_v = decodeMsg(msg);
            MotorVelocity motor_v = doIK(cmd_v);
            SendCommandsToMotors(motor_v);
            MotorVelocity motor_v_measured = ReadMotorSpeeds();
            CartVelocity robot_v_measured = doFK(motor_v_measured);
            ReportRobotVelocity(robot_v_measured);
        }
    }
}
