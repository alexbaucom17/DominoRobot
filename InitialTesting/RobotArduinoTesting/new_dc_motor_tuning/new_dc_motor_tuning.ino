#include "constants.h"
#include "Motor.h"

double Kp = 3;
double Ki = 0.1;
double Kd = 0.1;
Motor m = Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENC_A_3, PIN_ENC_B_3, Kp, Ki, Kd);

int step = 0;
unsigned long prevTime = millis();
int waitTime = 1000;

double speed = 3.14;

void setup()
{
    pinMode(PIN_ENABLE_3, OUTPUT);
    digitalWrite(PIN_ENABLE_3, HIGH);
    Serial.begin(115200);
    //Serial.println("Start motor tuning");
}


void loop()
{
    if (step == 0)
    {   
        if (millis() - prevTime > waitTime)
        {
            m.setCommand(speed);
            step = 1;
            prevTime = millis();
            waitTime = 2000;
            //Serial.println("Onto step 1");
        }
    }
    else if (step == 1)
    {
        if (millis() - prevTime > waitTime)
        {
            m.setCommand(0);
            step = 2;
            prevTime = millis();
            waitTime = 2000;
            //Serial.println("Onto step 2");
        }
    }
    else if (step == 2)
    {
        if (millis() - prevTime > waitTime)
        {
            m.setCommand(-1*speed);
            step = 3;
            prevTime = millis();
            waitTime = 2000;
           // Serial.println("Onto step 3");
        }
    }
    else if (step == 3)
    {
        if (millis() - prevTime > waitTime)
        {
            m.setCommand(0);
            step = 0;
            prevTime = millis();
            waitTime = 2000;
           // Serial.println("Onto step 0");
        }
    }

  m.runLoop();

}
