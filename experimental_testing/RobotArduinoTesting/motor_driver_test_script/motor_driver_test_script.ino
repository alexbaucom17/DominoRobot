#include "ClearCore.h"


#define LIFTER_MOTOR ConnectorM3
#define INCREMENTAL_UP_PIN DI7
#define INCREMENTAL_DOWN_PIN DI6
#define LATCH_SERVO_PIN IO0 // Only IO0 does pwm
#define HOMING_SWITCH_PIN IO4

#define LIFTER_STEPS_PER_REV 800

#define LIFTER_MAX_VEL 7    // revs/sec
#define LIFTER_MAX_ACC 10  // rev/sec^2

// Max num steps ~51000

#define SAFETY_MAX_POS 120  // Revs, Sanity check on desired position to make sure it isn't larger than this
#define SAFETY_MIN_POS 0 // Revs, Sanity check on desired position to make sure it isn't less than this

#define LATCH_ACTIVE_MS 2000
#define LATCH_OPEN_DUTY_CYCLE 50
#define LATCH_CLOSE_DUTY_CYCLE 200


void setup() 
{
    
    pinMode(INCREMENTAL_UP_PIN, INPUT);
    pinMode(INCREMENTAL_DOWN_PIN, INPUT);
    pinMode(HOMING_SWITCH_PIN, INPUT);
    pinMode(LATCH_SERVO_PIN, OUTPUT);

    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    
    LIFTER_MOTOR.VelMax(LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
    LIFTER_MOTOR.AccelMax(LIFTER_MAX_ACC*LIFTER_STEPS_PER_REV);
    LIFTER_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    LIFTER_MOTOR.PolarityInvertSDEnable(true);
    LIFTER_MOTOR.EnableRequest(true);

    Serial.begin(115200);
    Serial.println("Test script starting");
}


void test_motor()
{
    bool vel_up = digitalRead(INCREMENTAL_UP_PIN);
    bool vel_down = digitalRead(INCREMENTAL_DOWN_PIN);
    Serial.print("Up: ");
    Serial.print(vel_up);
    Serial.print(", down: ");
    Serial.println(vel_down);

    if(vel_up)
    {
        LIFTER_MOTOR.MoveVelocity(LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
        Serial.println(LIFTER_MOTOR.PositionRefCommanded());
    }
    else if(vel_down)
    {
        LIFTER_MOTOR.MoveVelocity(-1*LIFTER_MAX_VEL*LIFTER_STEPS_PER_REV);
        Serial.println(LIFTER_MOTOR.PositionRefCommanded());
    }  
    else 
    {
        LIFTER_MOTOR.MoveStopAbrupt();
    }
}

void test_homing_switch()
{
    bool switch_active = !digitalRead(HOMING_SWITCH_PIN);

    if(switch_active) {
      Serial.print("Homing switch: ");
     Serial.println(switch_active);
      LIFTER_MOTOR.PositionRefSet(0);
    }
}

void test_servo()
{
    Serial.println("Servo open");
    analogWrite(LATCH_SERVO_PIN, LATCH_OPEN_DUTY_CYCLE);
    delay(1000);
    Serial.println("Servo closed");
    analogWrite(LATCH_SERVO_PIN, LATCH_CLOSE_DUTY_CYCLE);
    delay(1000);
}

void loop()
{
    test_motor();
    test_homing_switch();
    //test_servo();
    delay(100);
}
