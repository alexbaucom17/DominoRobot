// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>

#define STEP_PIN 2
#define DIR_PIN 1   // Note that the DIR pin is actually controlled by the mster arduino so this won't be connected to anything
#define STEPS_PER_REV 1024 // How many steps the motor takes per revolution
#define VEL_PIN A0

#define MILLIS_BETWEEN_POLLING 10
#define MILLIS_BETWEEN_PRINTING 500


// This is assuming a mapping of 0V -> 5V = 0 steps/sec -> max_vel steps/sec
const int max_vel = 100; //steps/second
const int analog_resolution = 1024; // how many discrete steps available in analogRead
const float voltage_to_speed = static_cast<float>(max_vel) / static_cast<float>(analog_resolution);

AccelStepper motor;
int voltage;
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
      vel = voltage_to_speed * voltage;
      motor.setSpeed(vel);
      prevMillisRead = millis();

      if(millis() - prevMillisPrint > MILLIS_BETWEEN_PRINTING)
      {
       
       Serial.print(count);
       Serial.print(", ");
       Serial.print(voltage);
       Serial.print(", ");
       Serial.println(vel); 
       prevMillisPrint = millis();
       count = 0;
      }
      
    }
    
    motor.runSpeed();
    count++;
    
}
