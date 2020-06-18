#include "BaseController.h"
#include "constants.h"

BaseController::BaseController(StatusUpdater& statusUpdater, HardwareSerial& debug)
: statusUpdater_(statusUpdater),
  debug_(debug),
  load_phase_(0),
  load_timer_(0)
{
    pinMode(PIN_SENSOR_1, INPUT_PULLUP);
    pinMode(PIN_SENSOR_2, INPUT_PULLUP);
    pinMode(PIN_SENSOR_3, INPUT_PULLUP);
    pinMode(PIN_SENSOR_4, INPUT_PULLUP);

    analogWrite(PIN_MOTOR_PWM, PWM_STILL);
    pinMode(PIN_MOTOR_PWM, OUTPUT);
}

void BaseController::load()
{
    load_phase_ = 1;
    load_timer_ = millis();
}

void BaseController::estop()
{
    load_phase_ = 0;
}

void BaseController::update()
{
    // Update sensor values - negate due to input pullup
    bool val1 = !digitalRead(PIN_SENSOR_1);
    bool val2 = !digitalRead(PIN_SENSOR_2);
    bool val3 = !digitalRead(PIN_SENSOR_3);
    bool val4 = !digitalRead(PIN_SENSOR_4);
    statusUpdater_.updateSensors(val1, val2, val3, val4);

    // Update motor

    // Motor still
    if(load_phase_ == 0)
    {
        analogWrite(PIN_MOTOR_PWM, PWM_STILL);
        load_timer_ = millis();
    }

    // Motor loading forward
    else if(load_phase_ == 1)
    {
        if(millis() - load_timer_ < LOAD_FWD_TIME_MS)
        {
            analogWrite(PIN_MOTOR_PWM, PWM_FWD);
        }
        else
        {
            load_phase_ += 1;
            load_timer_ = millis();
        }
    }

    // Waiting for load
    else if(load_phase_ == 2)
    {
        if(millis() - load_timer_ < LOAD_PAUSE_TIME_MS)
        {
            // Nothing to do while waiting...
        }
        else
        {
            load_phase_ += 1;
            load_timer_ = millis();
        }
    }

    // Motor loading backwards
    else if(load_phase_ == 3)
    {
        if(millis() - load_timer_ < LOAD_BKWD_TIME_MS)
        {
            analogWrite(PIN_MOTOR_PWM, PWM_BKWD);
        }
        else
        {
            load_phase_ = 0;
        }
    }

}