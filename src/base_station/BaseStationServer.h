#ifndef BaseStationServer_h
#define BaseStationServer_h

#include <HardwareSerial.h>
#include "SimpleServer.h"

class BaseStationServer : public SimpleServer
{
  public:
   
    // Constructor
    BaseStationServer(HardwareSerial& serial, HardwareSerial& debug);

  private:

    virtual COMMAND getCommand(String message) override;

};



#endif
