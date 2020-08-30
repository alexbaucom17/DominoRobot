/*
* Robot server that handles gettings messages from master
* and responding correctly
*/

#ifndef RobotServer_h
#define RobotServer_h

#include <string>
#include <memory>

#include "constants.h"
#include "sockets/SocketMultiThreadWrapperBase.h"
#include "StatusUpdater.h"

#define START_CHAR '<'
#define END_CHAR '>'

class RobotServer
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
    
    RobotServer(StatusUpdater& statusUpdater);

    COMMAND oneLoop();

    RobotServer::PositionData getMoveData();

    RobotServer::PositionData getPositionData();

    RobotServer::VelocityData getVelocityData();

  private:
    PositionData moveData_;
    PositionData positionData_;
    VelocityData velocityData_;
    StatusUpdater& statusUpdater_;

    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;
    SocketMultiThreadWrapperBase* socket_;

    COMMAND getCommand(std::string message);
    void sendMsg(std::string msg, bool print_debug=true);
    void sendAck(std::string data);
    void sendErr(std::string data);
    std::string getAnyIncomingMessage();
    std::string cleanString(std::string message);
    void printIncomingCommand(std::string message);
    void sendStatus();

};



#endif
