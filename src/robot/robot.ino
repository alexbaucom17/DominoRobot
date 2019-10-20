/* Requires libraries:
*  Filters: https://github.com/JonHub/Filters
*  PID: https://playground.arduino.cc/Code/PIDLibrary/
*  Encoder: https://www.pjrc.com/teensy/td_libs_Encoder.html
*  ArduinoJson: https://arduinojson.org
*/

#include "globals.h"
#include "RobotServer.h"
#include "Motor.h"

const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

unsigned long startTime;
unsigned long prevTime;
RobotServer::COMMAND curCmd = RobotServer::COMMAND::NONE;
#define targetDelta 15
#define targetTotalTime 3000

Motor motors[4] = 
{ 
    Motor(PIN_PWM_1, PIN_DIR_1, PIN_ENCA_1, PIN_ENCB_1, Kp, Ki, Kd),
    Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENCA_2, PIN_ENCB_2, Kp, Ki, Kd),
    Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENCA_3, PIN_ENCB_3, Kp, Ki, Kd),
    Motor(PIN_PWM_4, PIN_DIR_4, PIN_ENCA_4, PIN_ENCB_4, Kp, Ki, Kd)
};

RobotServer server = RobotServer(Serial3, Serial);

unsigned long start1;
bool moveSet = false;

void setup()
{

    // Communication with the host computer
    Serial.begin(115200); 
    Serial.println("Robot starting");
    start1 = millis();

        // Setup pin modes
    pinMode(PIN_ENABLE,OUTPUT);

}

void setMotorCommand(int motor_num, float rps)
{
    int curDirection = 1;
    float curSpeed = curDirection * rps;  
    motors[motor_num].setCommand(curSpeed);
}

void runMotors()
{
    unsigned long curTime = millis();
    if(curTime - prevTime > targetDelta)
    {
        //Serial.print("Running motors: ");
        //Serial.println(curTime - prevTime);
        for(int i = 0; i < 4; i++)
        {
            motors[i].runLoop();
        }
        prevTime = millis();
    }
}



void loop() 
{
    // Check for new command
    // RobotServer::COMMAND newCmd = server.oneLoop();

    // if(newCmd != RobotServer::COMMAND::NONE)
    // {
    //     Serial.print("New command: ");
    //     Serial.println(newCmd);
    // }

    RobotServer::COMMAND newCmd = RobotServer::COMMAND::NONE;
    if (!moveSet && millis() - start1 > 5000)
    {
        newCmd = RobotServer::COMMAND::MOVE;
        curCmd = RobotServer::COMMAND::NONE;
        moveSet = true;
        Serial.println("Move set");
    }

    if(newCmd == RobotServer::COMMAND::MOVE && curCmd == RobotServer::COMMAND::NONE)
    {
        curCmd = newCmd;
        startTime = millis();
        float rps = 0.5;
        for(int i = 0; i < 4; i++)
        {
            setMotorCommand(i, rps);
        }
        digitalWrite(PIN_ENABLE, 1);
        Serial.println("Starting motors");
    }

    // Service motors
    if(curCmd == RobotServer::COMMAND::MOVE)
    {
        if(millis() - startTime < targetTotalTime)
        {
            runMotors();
        }
        else
        {
            Serial.println("Stopping motors");
            curCmd = RobotServer::COMMAND::NONE;
            digitalWrite(PIN_ENABLE, 0);
        } 
    }
}
