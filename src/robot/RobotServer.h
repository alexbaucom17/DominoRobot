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
       MOVE,
       MOVE_REL,
       MOVE_FINE,
       PLACE_TRAY,
       LOAD_TRAY,
       INITIALIZE_TRAY,
       POSITION,
       ESTOP,
       LOAD_COMPLETE,
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
    void printIncommingCommand(String message);

    const StatusUpdater& statusUpdater_;
    
    void sendMsg(String msg, bool print_debug=true);
    void sendAck(String data);
    void sendErr(String data);
    void sendStatus();

};



#endif
