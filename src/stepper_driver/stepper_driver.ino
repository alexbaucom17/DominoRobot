// Uses stepper driver library from here: https://github.com/DIMRobotics/ArduinoStepperDriver
#include <StepperDriver.h>

#define STEP_PIN 5
#define DIR_PIN 1   // Note that the DIR pin is actually controlled by the mster arduino so this won't be connected to anything
#define STEPS_PER_REV 1024 // How many steps the motor takes per revolution
#define VEL_PIN 19


// This is assuming a mapping of 0V -> 5V = 0 steps/sec -> max_vel steps/sec
const int max_vel = 1024; //steps/second
const int analog_resolution = 1024; // how many discrete steps available in analogRead
const float voltage_to_speed = static_cast<float>(max_vel) / static_cast<float>(analog_resolution);

axis_t motor;
int voltage;
int32_t vel;
unsigned long prevMillis;

void setup() 
{
//    Serial.begin(115200);
    StepperDriver.init();
    motor = StepperDriver.newAxis(STEP_PIN, DIR_PIN, STEPS_PER_REV);
//    prevMillis = millis();
}

void loop()
{
//    Serial.print(millis() - prevMillis);
//    prevMillis = millis();
    voltage = analogRead(VEL_PIN);
    vel = static_cast<int32_t>(voltage_to_speed * voltage);
//    Serial.print(", ");
//    Serial.print(voltage);
//    Serial.print(", ");
//    Serial.println(vel);
    StepperDriver.write(motor, vel);
    delay(10); // Run loop to update velocity at 100 Hz
}
