// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>

#define STEP_PIN 2
#define DIR_PIN 1   // Note that the DIR pin is actually controlled by the mster arduino so this won't be connected to anything
#define VEL_PIN A0

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

AccelStepper motor;
int voltage;
float calibrated_voltage;
float vel;
unsigned long count = 0;
unsigned long prevMillisRead;
unsigned long prevMillisPrint;

void setup() 
{
    Serial.begin(115200);
    motor = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    motor.setMaxSpeed(max_vel);
    motor.setAcceleration(max_vel); // Don't cap acceleration here, we handle accel in the master code
    motor.setMinPulseWidth(10); // Min from docs is 2.5 microseconds
    prevMillisRead = millis();
    prevMillisPrint = millis();
}

void loop()
{

    if(millis() - prevMillisRead > MILLIS_BETWEEN_POLLING)
    {
      voltage = analogRead(VEL_PIN);
      calibrated_voltage = calibration_slope * static_cast<float>(voltage) + calibration_offset;
      vel =  voltage_to_speed * calibrated_voltage;
      if(voltage < VOLTAGE_DEADBAND)
      {
        vel = 0;
      }
      motor.setSpeed(vel);
      prevMillisRead = millis();

      if(millis() - prevMillisPrint > MILLIS_BETWEEN_PRINTING)
      {
       
       Serial.print(count);
       Serial.print(", ");
       Serial.print(voltage);
       Serial.print(", ");
       Serial.print(calibrated_voltage);
       Serial.print(", ");
       Serial.println(vel); 
       prevMillisPrint = millis();
       count = 0;
      }
      
    }
    
    motor.runSpeed();
    count++;
    
}
