#ifndef SimpleServer_h
#define SimpleServer_h

#include <HardwareSerial.h>
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
    SimpleServer(HardwareSerial& serial, HardwareSerial& debug);

    virtual ~SimpleServer();

    void begin();

    COMMAND oneLoop();

  protected:

    HardwareSerial& serial_;
    HardwareSerial& debug_;

    void sendMsg(String msg, bool print_debug=true);
    void sendAck(String data);
    void sendErr(String data);
    String getAnyIncomingMessage();
    String cleanString(String message);
    void printIncommingCommand(String message);
    
    virtual COMMAND getCommand(String message)=0;

  private:

    bool clientConnected_;
    bool wifiConnected_;
    bool recvInProgress_;
    int recvIdx_;
    String buffer_;



};


#endif
