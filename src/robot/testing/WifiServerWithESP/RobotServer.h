/*
* Robot server that handles gettings messages from master
* and responding correctly
*/

#ifndef RobotServer_h
#define RobotServer_h

#include <HardwareSerial>

class RobotServer
{
  public:

    enum COMMAND
    {
       NONE,
       MOVE,
       PLACE,
       DOCK,
       UNDOCK,
       DROPOFF,
       PICKUP 
    };
    
    // Constructor
    RobotServer(HardwareSerial serial, HardwareSerial debug);

    int oneLoop();

  private:

    HardwareSerial serial_;
    HardwareSerial debug_;
    bool clientConnected_;

    string getAnyIncomingMessage();
    void sendMessage(string message);

}



#endif