#include "RobotServer.h"


RobotServer server = RobotServer(Serial3, Serial);


void setup() {
  // put your setup code here, to run once:

  // Communication with the host computer
  Serial.begin(9600); 
  Serial.println("Wifi client starting"); 

}

void loop() {
  // put your main code here, to run repeatedly:

  int cmd = server.oneLoop();
  delay(50);

}
