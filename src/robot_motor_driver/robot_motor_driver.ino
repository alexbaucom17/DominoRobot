#include "ClearCore.h"
#include "SerialComms.h"
#include <math.h>

// From https://teknic-inc.github.io/ClearCore-library/ArduinoRef.html
// Serial = USB
// Serial0 + Serial1 = COM ports

#define PRINT_DEBUG false
#define USE_FAKE_MOTOR false

// Globals
SerialComms comm(Serial);

// --------------------------------------------------
//                Base parameters
// --------------------------------------------------

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

#if USE_FAKE_MOTOR
float MOTOR_FRONT_LEFT_FAKE = 0;
float MOTOR_FRONT_RIGHT_FAKE = 0;
float MOTOR_REAR_CENTER_FAKE = 0;
#else
// Motor mapping
#define MOTOR_FRONT_LEFT ConnectorM1
#define MOTOR_FRONT_RIGHT ConnectorM2
#define MOTOR_REAR_CENTER ConnectorM0
#endif

// --------------------------------------------------
//                Lifter parameters
// --------------------------------------------------

#define LIFTER_MOTOR ConnectorM3
#define INCREMENTAL_UP_PIN DI7
#define INCREMENTAL_DOWN_PIN DI6
#define LATCH_SERVO_PIN IO0 // Only IO0 does pwm
#define HOMING_SWITCH_PIN IO4

#define LIFTER_STEPS_PER_REV 800

#define LIFTER_MAX_VEL 2  // revs/sec
#define LIFTER_MAX_ACC 10  // rev/sec^2

#define SAFETY_MAX_POS 120  // Revs, Sanity check on desired position to make sure it isn't larger than this
#define SAFETY_MIN_POS 0 // Revs, Sanity check on desired position to make sure it isn't less than this

#define LATCH_ACTIVE_MS 1000
#define LATCH_OPEN_DUTY_CYCLE 50
#define LATCH_CLOSE_DUTY_CYCLE 200

// --------------------------------------------------
//                Helper structs
// --------------------------------------------------

enum MODE
{
    NONE,
    MANUAL_VEL,
    AUTO_POS,
    HOMING,
    LATCH_OPEN,
    LATCH_CLOSE,
};

struct Command
{
    int abs_pos;
    bool home;
    bool valid;
    bool stop;
    bool latch_open;
    bool latch_close;
    bool status_req;
};

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


// --------------------------------------------------
//                Lifter control functions
// --------------------------------------------------

MODE activeMode = MODE::NONE;
unsigned long prevLatchMillis = millis();

void lifter_setup() 
{
    
    pinMode(INCREMENTAL_UP_PIN, INPUT);
    pinMode(INCREMENTAL_DOWN_PIN, INPUT);
    pinMode(HOMING_SWITCH_PIN, INPUT);
    pinMode(LATCH_SERVO_PIN, OUTPUT);

    LIFTER_MOTOR.EnableRequest(false);
    LIFTER_MOTOR.VelMax(LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
    LIFTER_MOTOR.AccelMax(LIFTER_MAX_ACC*LIFTER_STEPS_PER_REV);
}


// Possible inputs are
// "home", "stop", or an integer representing position to move to
// "open" or "close" controls the latch servo
// "status_req" will request a status of the mode
// Note that this assumes the message has already been validated
Command decodeLifterMsg(String msg_in)
{
    // Strip identifier
    String msg = msg_in.substring(5);
    Command c = {0, false, false, false, false, false, false};

#if PRINT_DEBUG
    comm.send("DEBUG Incoming lifter message: " + msg);
#endif

    if(msg == "home")
    {
        c.home = true;
        c.valid = true;
    }
    else if(msg == "stop")
    {
        c.stop = true;
        c.valid = true;
    }
    else if (msg == "open")
    {
        c.latch_open = true;
        c.valid = true;
    }
    else if (msg == "close")
    { 
        c.latch_close = true;
        c.valid = true;
    }
    else if (msg == "status_req")
    {
      c.valid = true;
      c.status_req = true;
    }
    else if (msg.startsWith("pos:"))
    {
        String pos_msg = msg.substring(4);
        c.abs_pos = pos_msg.toInt();
        c.valid = true;
    }

    return c;
}

void lifter_update(String msg)
{
    // Verify if incoming message is for lifter
    Command inputCommand = {0, false, false, false, false, false};
    bool valid_msg = false;
    if(msg.length() > 0 && msg.startsWith("lift:")) 
    { 
        valid_msg = true; 
    }

    // Parse command if it was valid
    if(valid_msg) 
    {
        inputCommand = decodeLifterMsg(msg);
    }
    
    // Read inputs for manual mode
    bool vel_up = digitalRead(INCREMENTAL_UP_PIN);
    bool vel_down = digitalRead(INCREMENTAL_DOWN_PIN);

    // For debugging only
    // Serial.print("Vel up: ");
    // Serial.print(vel_up);
    // Serial.print(" Vel dn: ");
    // Serial.println(vel_down);

    // If we got a stop command, make sure to handle it immediately
    if (inputCommand.valid && inputCommand.stop)
    {
        activeMode = MODE::NONE;
    }
    // For any other command, we will only handle it when there are no other active commands
    else if (activeMode == MODE::NONE)
    {
        // Figure out what mode we are in
        if(vel_up || vel_down)
        {
            activeMode = MODE::MANUAL_VEL;
            if(vel_up)
            {
                LIFTER_MOTOR.MoveVelocity(-1*LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
            }
            else if(vel_down)
            {
                LIFTER_MOTOR.MoveVelocity(LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
            }   
        }
        else if(inputCommand.valid && inputCommand.home)
        {
            activeMode = MODE::HOMING;
            LIFTER_MOTOR.MoveVelocity(-1*LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
        }
        else if(inputCommand.valid && inputCommand.latch_open)
        {
            activeMode = MODE::LATCH_OPEN;
            analogWrite(LATCH_SERVO_PIN, LATCH_OPEN_DUTY_CYCLE);
            prevLatchMillis = millis();
        }
        else if(inputCommand.valid && inputCommand.latch_close)
        {
            activeMode = MODE::LATCH_CLOSE;
            analogWrite(LATCH_SERVO_PIN, LATCH_CLOSE_DUTY_CYCLE);
            prevLatchMillis = millis();
        }
        else if (inputCommand.valid && inputCommand.status_req)
        {
          // Nothing to do here
        }
        else if(inputCommand.valid)
        {
            if(inputCommand.abs_pos <= SAFETY_MAX_POS && inputCommand.abs_pos >= SAFETY_MIN_POS)
            {
                activeMode = MODE::AUTO_POS;
                long target = inputCommand.abs_pos;
                LIFTER_MOTOR.Move(target*LIFTER_STEPS_PER_REV, StepGenerator::MOVE_TARGET_ABSOLUTE);
            }
        }
    }    


    // Handle continous updates for each mode
    String status_str = "lift:none";
    if(activeMode == MODE::MANUAL_VEL)
    {
        status_str = "lift:manual";
        if(!(vel_up || vel_down))
        {
            activeMode = MODE::NONE;
        }
    }
    else if(activeMode == MODE::AUTO_POS)
    {
        status_str = "lift:pos";
        if(LIFTER_MOTOR.StepsComplete())
        {
            activeMode = MODE::NONE;
        }
    }
    else if(activeMode == MODE::HOMING)
    {
        if(!digitalRead(HOMING_SWITCH_PIN))
        {
            LIFTER_MOTOR.MoveStopAbrupt();
            LIFTER_MOTOR.PositionRefSet(0);
            activeMode = MODE::NONE;
        }
        status_str = "lift:homing";
    }
    else if (activeMode == MODE::LATCH_CLOSE)
    {
        status_str = "lift:close";
        if(millis() - prevLatchMillis > LATCH_ACTIVE_MS)
        {
            activeMode = MODE::NONE;
        }
    }
    else if (activeMode == MODE::LATCH_OPEN)
    {
        status_str = "lift:open";
        if(millis() - prevLatchMillis > LATCH_ACTIVE_MS)
        {
            activeMode = MODE::NONE;
        }
    }
    else if (activeMode == MODE::NONE)
    {
        LIFTER_MOTOR.MoveStopAbrupt();
    }

    // Will send back one of [none, manual, pos, homing, open, close]
    if (valid_msg) 
    {
        comm.send(status_str);
    }
    // For debugging only
    // Serial.println("");
}



// --------------------------------------------------
//                Base control functions
// --------------------------------------------------


CartVelocity decodeBaseMsg(String msg)
{
#if PRINT_DEBUG
    comm.send("DEBUG Incoming base message: " + msg);
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
    motors.v0 = -1/WHEEL_RADIUS * (-1 * sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
    motors.v1 = -1/WHEEL_RADIUS * (     sq3 / 2 * cmd.vx  + 0.5 * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
    motors.v2 = -1/WHEEL_RADIUS * (                       - 1   * cmd.vy + WHEEL_DIST_FROM_CENTER * cmd.va) * BELT_RATIO;
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

#if USE_FAKE_MOTOR
    MOTOR_FRONT_LEFT_FAKE  = motors.v0 * radsPerSecondToStepsPerSecond;
    MOTOR_FRONT_RIGHT_FAKE = motors.v1 * radsPerSecondToStepsPerSecond;
    MOTOR_REAR_CENTER_FAKE = motors.v2 * radsPerSecondToStepsPerSecond;
#else
    MOTOR_FRONT_LEFT.MoveVelocity(motors.v0 * radsPerSecondToStepsPerSecond);
    MOTOR_FRONT_RIGHT.MoveVelocity(motors.v1 * radsPerSecondToStepsPerSecond);
    MOTOR_REAR_CENTER.MoveVelocity(motors.v2 * radsPerSecondToStepsPerSecond);
#endif
}

MotorVelocity ReadMotorSpeeds()
{
    MotorVelocity measured;

#if USE_FAKE_MOTOR
     measured.v0 = MOTOR_FRONT_LEFT_FAKE / radsPerSecondToStepsPerSecond;
     measured.v1 = MOTOR_FRONT_RIGHT_FAKE / radsPerSecondToStepsPerSecond;
     measured.v2 = MOTOR_REAR_CENTER_FAKE / radsPerSecondToStepsPerSecond;
#else
    measured.v0 = MOTOR_FRONT_LEFT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    measured.v1 = MOTOR_FRONT_RIGHT.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
    measured.v2 = MOTOR_REAR_CENTER.VelocityRefCommanded() / radsPerSecondToStepsPerSecond;
#endif

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
    robot_measured.vx = -rOver3 * (-1 * sq3 * wheel_speed.v0 + sq3 * wheel_speed.v1);
    robot_measured.vy = -rOver3 * ( wheel_speed.v0 + wheel_speed.v1 - 2.0 * wheel_speed.v2); 
    robot_measured.va = -rOver3 / WHEEL_DIST_FROM_CENTER * (wheel_speed.v0 + wheel_speed.v1 + wheel_speed.v2);
#if PRINT_DEBUG
    comm.send("DEBUG After FK: " + robot_measured.toString());
#endif
    return robot_measured;

}

void ReportRobotVelocity(CartVelocity robot_v_measured)
{
    char msg[32];
    sprintf(msg, "base:%.3f,%.3f,%.3f",robot_v_measured.vx, robot_v_measured.vy, robot_v_measured.va);
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

void base_setup()
{    
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

bool isValidBaseMessage(String msg)
{
    bool result = false;
    if(msg.length() > 0 && msg.startsWith("base:")) 
    { 
        result = true; 
    }
    return result;
}


void base_update(String msg)
{    
    if(!isValidBaseMessage(msg)) { return; }

    // Strip base identifier
    String new_msg = msg.substring(5);
    
    if(!handlePowerRequests(new_msg))
    {
        CartVelocity cmd_v = decodeBaseMsg(new_msg);
        MotorVelocity motor_v = doIK(cmd_v);
        SendCommandsToMotors(motor_v);
        MotorVelocity motor_v_measured = ReadMotorSpeeds();
        CartVelocity robot_v_measured = doFK(motor_v_measured);
        ReportRobotVelocity(robot_v_measured);
    }
}


// --------------------------------------------------
//                Top level functions
// --------------------------------------------------

void setup()
{
    Serial.begin(115200);

    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    base_setup();
    lifter_setup();
}

void loop()
{
    String msg = comm.rcv();
    base_update(msg);
    lifter_update(msg);
}
