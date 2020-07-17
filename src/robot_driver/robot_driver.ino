#include "SerialComms.h"


void setup()
{
    Serial2.begin(115200);
    SerialComms comm(Serial1, Serial2);
}

void loop()
{
    comm.send("Test");
    delay(1000)
    String msg = comm.rcv();
    Serial2.print(msg);
}