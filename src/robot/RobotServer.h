/*
* Robot server that handles gettings messages from master
* and responding correctly
*/

#ifndef RobotServer_h
#define RobotServer_h

#include <HardwareSerial.h>
#include "SimpleServer.h"
#include "StatusUpdater.h"

class RobotServer : public SimpleServer
{
  public:

    struct PositionData
    {
      float x;
      float y;
      float a;
    };
    
    // Constructor
    RobotServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater);

    RobotServer::PositionData getMoveData();

    RobotServer::PositionData getPositionData();

  private:
    PositionData moveData_;
    PositionData positionData_;
    const StatusUpdater& statusUpdater_;

    virtual COMMAND getCommand(String message) override;

    void sendStatus();

};



#endif
