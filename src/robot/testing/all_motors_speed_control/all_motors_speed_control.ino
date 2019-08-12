/* Requires libraries:
 *  Filters: https://github.com/JonHub/Filters
 *  PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  Encoder: https://www.pjrc.com/teensy/td_libs_Encoder.html
 */

#include "Motor.h"

// Pinouts
#define PIN_ENABLE 52
#define PIN_FR_PWM 3
#define PIN_FR_DIR 49
#define PIN_FL_PWM 2
#define PIN_FL_DIR 51
#define PIN_BR_PWM 5
#define PIN_BR_DIR 50
#define PIN_BL_PWM 6
#define PIN_BL_DIR 48

#define PIN_FR_ENC_A 20
#define PIN_FR_ENC_B 35
#define PIN_FL_ENC_A 21
#define PIN_FL_ENC_B 41
#define PIN_BR_ENC_A 18
#define PIN_BR_ENC_B 23
#define PIN_BL_ENC_A 19
#define PIN_BL_ENC_B 29

#define VEL_FILTER_FREQ 0.5 // HZ

const double Kp = 100;
const double Ki = 1;
const double Kd = 1;

Motor allMotors[4] = {
Motor(PIN_FR_PWM, PIN_FR_DIR, PIN_FR_ENC_A, PIN_FR_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ),
Motor(PIN_FL_PWM, PIN_FL_DIR, PIN_FL_ENC_A, PIN_FL_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ),
Motor(PIN_BR_PWM, PIN_BR_DIR, PIN_BR_ENC_A, PIN_BR_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ),
Motor(PIN_BL_PWM, PIN_BL_DIR, PIN_BL_ENC_A, PIN_BL_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ) };

const int MOTOR_DIRECTIONS[4][4] = 
{
  {1, 1, 1, 1},   //Forward
  {-1, -1, -1, -1},   //Backward
  {1, -1, -1, 1},   //Left
  {-1, 1, 1, -1}   //Right
//  {1, 0, 1, 0},   //Forward-Left
//  {1, 0, 0, 1},   //Forward-Right
//  {0, 1, 0, 1},   //Backward-Left
//  {0, 1, 1, 0}    //Backward-Right
};

const float SPEED[4] = {2, 1, 1, 0.5}; // scale factor for rev/s


void setup() 
{
  // Setup pin modes
  pinMode(PIN_ENABLE,OUTPUT);
  
  Serial.begin(250000);
}

int waitForInput()
{
  int motion = 0;
  bool gotInput = false;
  while(!gotInput)
  {
    if(Serial.available() > 0)
    {
//      Serial.println("Serial available");
       int incomingByte = Serial.read();
//      Serial.print("Read: ");
//      Serial.println(incomingByte);
      if(incomingByte == 49)
      {
        motion = 1;
        gotInput = true;  
      }
      else if(incomingByte == 50)
      {
        motion = 2;
        gotInput = true;  
      }
      else if(incomingByte == 51)
      {
        motion = 3;
        gotInput = true;  
      }
      else if(incomingByte == 52)
      {
        motion = 4;
        gotInput = true;  
      }
      else
      {
        // Do nothing
      }
    }
  }
  return motion;  
}

void setMotorCommands(int motion)
{
  for(int i = 0; i < 4; i++)
  {
    int curDirection = MOTOR_DIRECTIONS[motion-1][i];
    float curSpeed = curDirection * SPEED[i];
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(", Direction ");
    Serial.print(curDirection);
    Serial.print(", Speed ");
    Serial.println(curSpeed);
    
    allMotors[i].setCommand(curSpeed);
  }
  digitalWrite(PIN_ENABLE, 1);
}

void runMotors()
{
  unsigned long prevTime = millis();
  unsigned long startTime = prevTime;
  int targetDelta = 20;
  int targetTotalTime = 2000;

  while (millis() - startTime < targetTotalTime)
  {
    if(millis() - prevTime > targetDelta)
    {
      prevTime = millis();
      for(int i = 0; i < 4; i++)
      {
        allMotors[i].runLoop();
      }
    }
  }
}

void loop() 
{

  int motion = waitForInput();

  setMotorCommands(motion);

  runMotors();

  // Stop motors
  digitalWrite(PIN_ENABLE, 0);
  
}
