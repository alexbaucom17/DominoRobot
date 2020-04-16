// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>

#define STEP_PIN_0 2
#define STEP_PIN_1 3
#define STEP_PIN_2 4
#define STEP_PIN_3 5
#define VEL_PIN_0 A0
#define VEL_PIN_1 A1
#define VEL_PIN_2 A2
#define VEL_PIN_3 A3
#define DIR_PIN 6   // Note that the DIR pin is actually controlled by the mster arduino so this won't be connected to anything

#define MILLIS_BETWEEN_POLLING 10
#define MILLIS_BETWEEN_PRINTING 200
#define VOLTAGE_DEADBAND 5

// This is assuming a mapping of 0V -> 5V = 0 steps/sec -> max_vel steps/sec
const int max_vel = 1600; //steps/second
const int analog_resolution = 1024; // how many discrete steps available in analogRead
const float voltage_to_speed = static_cast<float>(max_vel) / static_cast<float>(analog_resolution);
// Calibrated using spreadsheet in docs folder - just fit a line to voltages
// Used only one motor with the same setup for all of it  hopefully this is representative enough of other
// motors and setups....
const float calibration_slope = 0.9510;
const float calibration_offset = 1.952;
const int step_pins[4] = {STEP_PIN_0, STEP_PIN_1, STEP_PIN_2, STEP_PIN_3};
const int vel_pins[4] = {VEL_PIN_0, VEL_PIN_1, VEL_PIN_2, VEL_PIN_3};

AccelStepper motors[4];
int voltage;
float calibrated_voltage;
float vel;
unsigned long count = 0;
unsigned long prevMillisRead;
unsigned long prevMillisPrint;

void setup() 
{
    Serial.begin(115200);
    prevMillisRead = millis();
    prevMillisPrint = millis();
    
    for(int i = 0; i < 4; i++)
    {
      motors[i] = AccelStepper(AccelStepper::DRIVER, step_pins[i], DIR_PIN);
      motors[i].setMaxSpeed(max_vel);
      motors[i].setAcceleration(max_vel); // Don't cap acceleration here, we handle accel in the master code
      motors[i].setMinPulseWidth(10); // Min from docs is 2.5 microseconds
    }
}

void loop()
{

    if(millis() - prevMillisRead > MILLIS_BETWEEN_POLLING)
    {
      for (int i = 0; i < 4; i++)
      {
        voltage = analogRead(vel_pins[i]);
        calibrated_voltage = calibration_slope * static_cast<float>(voltage) + calibration_offset;
        vel =  voltage_to_speed * calibrated_voltage;
        if(voltage < VOLTAGE_DEADBAND)
        {
          vel = 0;
        }
        motors[i].setSpeed(vel);
      }
      prevMillisRead = millis();

      // Note this doesn't work properly anymore with the for loops. might want to fix
//      if(millis() - prevMillisPrint > MILLIS_BETWEEN_PRINTING)
//      {
//       
//       Serial.print(count);
//       Serial.print(", ");
//       Serial.print(voltage);
//       Serial.print(", ");
//       Serial.print(calibrated_voltage);
//       Serial.print(", ");
//       Serial.println(vel); 
//       prevMillisPrint = millis();
//       count = 0;
//      }
      
    }

    for (int i = 0; i < 4; i++)
    {
      motors[i].runSpeed();
    }
    count++;
    
}
