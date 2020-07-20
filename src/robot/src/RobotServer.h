/*
* Robot server that handles gettings messages from master
* and responding correctly
*/

#ifndef RobotServer_h
#define RobotServer_h

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

    struct VelocityData
    {
      float vx;
      float vy;
      float va;
      float t;
    };
    
    // Constructor
    RobotServer(StatusUpdater& statusUpdater);

    RobotServer::PositionData getMoveData();

    RobotServer::PositionData getPositionData();

    RobotServer::VelocityData getVelocityData();

  private:
    PositionData moveData_;
    PositionData positionData_;
    VelocityData velocityData_;
    StatusUpdater& statusUpdater_;

    virtual COMMAND getCommand(std::string message) override;

    void sendStatus();

};



#endif
