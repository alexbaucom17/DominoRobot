#include "constants.h"
#include "Motor.h"

double Kp = 14;
double Ki = 0.1;
double Kd = 0.08;
Motor m = Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENC_A_2, PIN_ENC_B_2, Kp, Ki, Kd);

int step = 0;
unsigned long prevStepTime = millis();
unsigned long prevLoopTime = millis();
double cur_speed = 0;

double max_vel = 0.8;
double max_acc = 2;
double max_acc_time = 1000 * max_vel / max_acc;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void setup()
{
    pinMode(PIN_ENABLE_3, OUTPUT);
    digitalWrite(PIN_ENABLE_3, HIGH);
    Serial.begin(115200);
    //Serial.println("Start motor tuning");
}

double computeAcc(double dt, double acc)
{
  cur_speed += dt/1000.0 * acc;
  if (fabsf(cur_speed) > max_vel)
  {
    cur_speed = sgn(cur_speed) * max_vel;
  }
}

void runAccStep(int waitTime, double acc)
{
    unsigned long curTime = millis();
    unsigned long stepTime = curTime - prevStepTime;
    if (stepTime > waitTime)
    {
      step += 1;
      prevStepTime = millis();
      prevLoopTime = millis();
    }
    else
    {
        unsigned long loopTime = curTime - prevLoopTime;
        computeAcc(loopTime, acc);
        prevLoopTime = millis();
    }
}

double runCVStep(int waitTime, double vel)
{
    unsigned long curTime = millis();
    unsigned long stepTime = curTime - prevStepTime;
    if (stepTime > waitTime)
    {
      step += 1;
      prevStepTime = millis();
      prevLoopTime = millis();
    }
    else
    {
       cur_speed = vel;
    }
  
}

double runSteps()
{
    if (step == 0)
    {   
      int stepTime = max_acc_time;
      double stepAcc = max_acc;
      runAccStep(stepTime, stepAcc);
    }
    else if (step == 1)
    {
      int stepTime = 2000;
      runCVStep(stepTime, max_vel);
    }
    else if (step == 2)
    {
      int stepTime = max_acc_time;
      double stepAcc = -1*max_acc;
      runAccStep(stepTime, stepAcc);
    }
    else if (step == 3)
    {
      int stepTime = 2000;
      runCVStep(stepTime, 0);
    }
    else if (step == 4)
    {   
      int stepTime = max_acc_time;
      double stepAcc = -1 * max_acc;
      runAccStep(stepTime, stepAcc);
    }
    else if (step == 5)
    {
      int stepTime = 2000;
      runCVStep(stepTime, -1*max_vel);
    }
    else if (step == 6)
    {
      int stepTime = max_acc_time;
      double stepAcc = max_acc;
      runAccStep(stepTime, stepAcc);
    }
    else if (step == 7)
    {
      int stepTime = 2000;
      runCVStep(stepTime, 0);
    }
    else if (step >= 8)
    {
      step = 0;
    }
}


void loop()
{
  runSteps();
  m.setCommand(cur_speed);

  bool motor_pub = true;
  if(step == 3 || step == 7) {motor_pub  = false;}
  m.runLoop(motor_pub);
//  analogWrite(PIN_PWM_2, s);
//  Serial.println(s);
//  s += 1;
 delay(15);
}
