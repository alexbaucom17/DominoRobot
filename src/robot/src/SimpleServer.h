#ifndef SimpleServer_h
#define SimpleServer_h

#include <string>

#include "constants.h"
#include "SocketWrapper.h"

#define START_CHAR '<'
#define END_CHAR '>'

class SimpleServer
{
  public:
    
    // Constructor
    SimpleServer();

    virtual ~SimpleServer();

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

    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;

    SocketWrapper socket_;



};


#endif
