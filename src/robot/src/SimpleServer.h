#ifndef SimpleServer_h
#define SimpleServer_h

#include <string>

#include "constants.h"

#define START_CHAR '<'
#define END_CHAR '>'

/*
This is a base server class to use for the robot and base station
Due to arduino library funniness, I can't easily re-use the server
class for separate sketches, so this just puts as much common stuff in one file
that I will have to manually keep in sync. Then all of the specialized stuff can
go in the subclassed files
*/
class SimpleServer
{
  public:
    
    // Constructor
    SimpleServer();

    virtual ~SimpleServer();

    void begin();

    COMMAND oneLoop();

  protected:

    void sendMsg(std::string msg, bool print_debug=true);
    void sendAck(std::string data);
    void sendErr(std::string data);
    std::string getAnyIncomingMessage();
    std::string cleanString(std::string message);
    void printIncommingCommand(std::string message);
    
    virtual COMMAND getCommand(std::string message)=0;

  private:

    bool clientConnected_;
    bool wifiConnected_;
    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;



};


#endif
