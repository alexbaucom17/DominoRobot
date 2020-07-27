// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>

// LEFT/RIGHT is WRT standing at back of robot looking forward
#define STEP_PIN_LEFT 2
#define STEP_PIN_RIGHT 3
#define DIR_PIN_LEFT 4
#define DIR_PIN_RIGHT 5
#define INCREMENTAL_UP_PIN 6
#define INCREMENTAL_DOWN_PIN 7
#define POSITIONING_PIN_1 8
#define POSITIONING_PIN_2 9
#define POSITIONING_PIN_3 10
#define HOMING_SWITCH_PIN 11
#define FEEDBACK_PIN 12

#define MILLIS_BETWEEN_POLLING 10
#define MILLIS_BETWEEN_PRINTING 200

#define MAX_VEL 10  // steps/sec
#define MAX_ACC 100  // steps/sec^2

#define HOMING_DIST 10000 // This should be larger than the max distance the lifter can go

int positions[8] = {  0,      // Dont use - corresponds to no motion
                      100,    // Fully raised for loading
                      200,    // Lowered slightly for entering/exiting loading
                      500,    // Midway up for driving
                      1000,   // Lowered for placement
                      0,      // unused
                      0,      // unused
                      0 };    // Don't use - triggers homing

AccelStepper motors[2];
unsigned long count = 0;
unsigned long prevMillisRead;
unsigned long prevMillisPrint;
bool move_in_progress = false;

bool mode_vel_active = false;
bool mode_pos_active = false;
bool mode_homing_active = false;

void setup() 
{
    pinMode(INCREMENTAL_UP_PIN, INPUT_PULLUP);
    pinMode(INCREMENTAL_DOWN_PIN, INPUT_PULLUP);
    pinMode(POSITIONING_PIN_1, INPUT_PULLUP);
    pinMode(POSITIONING_PIN_1, INPUT_PULLUP);
    pinMode(POSITIONING_PIN_1, INPUT_PULLUP);
    pinMode(HOMING_SWITCH_PIN, INPUT_PULLUP);
    pinMode(FEEDBACK_PIN, OUTPUT);
    digitalWrite(FEEDBACK_PIN, 0);
    
    Serial.begin(115200);
    prevMillisRead = millis();
    prevMillisPrint = millis();
    
    motors[0] = AccelStepper(AccelStepper::DRIVER, STEP_PIN_LEFT, DIR_PIN_LEFT);
    motors[1] = AccelStepper(AccelStepper::DRIVER, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

    for(int i = 0; i < 2; i++)
    {
      motors[i].setMaxSpeed(MAX_VEL);
      motors[i].setAcceleration(MAX_ACC);
      motors[i].setMinPulseWidth(10); // Min from docs is 2.5 microseconds
    }
}

void loop()
{

    // Poll inputs and update our state
    if(millis() - prevMillisRead > MILLIS_BETWEEN_POLLING)
    {

      // Read inputs for vel
      bool vel_up = !digitalRead(INCREMENTAL_UP_PIN);
      bool vel_down = !digitalRead(INCREMENTAL_DOWN_PIN);

      // Read inputs for pos
      int pos_num = 0;
      pos_num += !digitalRead(POSITIONING_PIN_1) ? 1 : 0;
      pos_num += !digitalRead(POSITIONING_PIN_1) ? 2 : 0;
      pos_num += !digitalRead(POSITIONING_PIN_1) ? 4 : 0;

      // Figure out what mode we are in
      if(vel_up || vel_down)
      {
        mode_vel_active = true;
      }
      else if(pos_num > 0 && pos_num < 7)
      {
        mode_pos_active = true;
      }
      else if(pos_num == 7)
      {
        mode_homing_active = true;
      }
      else
      {
        mode_pos_active = false;
        mode_vel_active = false;
        mode_homing_active = false;
      }
      
      // Set motor speeds based on inputs and mode
      if(mode_vel_active)
      {
        if(vel_up)
        {
          motors[0].setSpeed(MAX_VEL);
          motors[1].setSpeed(MAX_VEL);
        }
        else if(vel_down)
        {
          motors[0].setSpeed(-1*MAX_VEL);
          motors[1].setSpeed(-1*MAX_VEL);
        }     
      }
      else if(mode_pos_active && !move_in_progress)
      {
        int target = positions[pos_num];
        motors[0].moveTo(target);
        motors[1].moveTo(target);
        move_in_progress = true;
      }
      else if(mode_homing_active && !move_in_progress)
      {
        motors[0].moveTo(HOMING_DIST);
        motors[1].moveTo(HOMING_DIST);
        move_in_progress = true;
      }
      else if(!move_in_progress)
      {
        // Stop motors if no move is in progress and no buttons are pushed
        motors[0].stop();
        motors[1].stop();        
      }

      prevMillisRead = millis();      
    }

    // Call motor update loop
    if(mode_vel_active)
    {
      motors[0].runSpeed();
      motors[1].runSpeed();
    }
    else if(mode_pos_active)
    {
      bool tmp1 = motors[0].run();
      bool tmp2 = motors[1].run();
      move_in_progress = tmp1 || tmp2;
    }
    else if(mode_homing_active)
    {
      motors[0].run();
      motors[1].run();
      if(!digitalRead(HOMING_SWITCH_PIN))
      {
        motors[0].stop();
        motors[1].stop();  
        move_in_progress = false;
        motors[0].setCurrentPosition(0);
        motors[1].setCurrentPosition(0);
      }
    }

    digitalWrite(FEEDBACK_PIN, move_in_progress || mode_vel_active);
    count++;
    
}
