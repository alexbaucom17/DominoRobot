// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>

#define STEP_PIN 2
#define DIR_PIN 1   // Note that the DIR pin is actually controlled by the mster arduino so this won't be connected to anything

#define MILLIS_BETWEEN_PRINTING 100
const int max_vel = 800; //steps/second

AccelStepper motor;
unsigned long count = 0;
unsigned long prevMillisPrint;

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting...");
    motor = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    motor.setMaxSpeed(max_vel);
    motor.setAcceleration(max_vel);
    motor.setMinPulseWidth(10); // Min from docs is 2.5 microseconds
    prevMillisPrint = millis();

    motor.move(800);
}

void loop()
{

    if(motor.isRunning() && millis() - prevMillisPrint > MILLIS_BETWEEN_PRINTING)
    {       
       Serial.print(motor.speed());
       Serial.print(", ");
       Serial.println(motor.currentPosition());
       prevMillisPrint = millis();
       count = 0;      
    }
    
    motor.run();
    count++;
    
}
