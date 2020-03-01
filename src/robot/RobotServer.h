/*
* Robot server that handles gettings messages from master
* and responding correctly
*/

#ifndef RobotServer_h
#define RobotServer_h

#include <HardwareSerial.h>
#include "StatusUpdater.h"

#define START_CHAR '<'
#define END_CHAR '>'

class RobotServer
{
  public:

    enum COMMAND
    {
       NONE,
       ERROR_NO_TYPE,
       ERROR_UNKOWN_TYPE,
       ERROR_BAD_JSON,
       MOVE,
       MOVE_REL,
       MOVE_FINE,
       PLACE,
       POSITION,
       STATUS,
       CHECK
    };

    struct PositionData
    {
      float x;
      float y;
      float a;
    };
    
    // Constructor
    RobotServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater);

    void begin();

    RobotServer::COMMAND oneLoop();

    RobotServer::PositionData getMoveData();

    RobotServer::PositionData getPositionData();

  private:

    HardwareSerial& serial_;
    HardwareSerial& debug_;
    bool clientConnected_;
    bool wifiConnected_;
    bool recvInProgress_;
    int recvIdx_;
    String buffer_;
    PositionData moveData_;
    PositionData positionData_;

    String getAnyIncomingMessage();
    String cleanString(String message);
    COMMAND getCommand(String message);

    const StatusUpdater& statusUpdater_;
    
    void sendMsg(String msg);
    void sendAck(String data);
    void sendErr(String data);
    void sendStatus();

};



#endif
