#include "SerialComms.h"

// From https://teknic-inc.github.io/ClearCore-library/ArduinoRef.html
// Serial = USB
// Serial0 + Serial1 = COM ports

SerialComms comm(Serial0, Serial);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    String msg = comm.rcv();
    if(!msg.empty())
    {
        CartVelocity cmd_v = decodeMsg(msg);
        MotorVelocity motor_v = doIK(cmd_v);
        SendCommandsToMotors(motor_v);
        MotorVelocity motor_v_measured = ReadMotorSpeeds();
        CartVelocity robot_v_measured = doFK(motor_v_measured);
        ReportRobotVelocity(robot_v_measured);
    }

}