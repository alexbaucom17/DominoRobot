#ifndef BaseStationServer_h
#define BaseStationServer_h

#include <HardwareSerial.h>
#include "SimpleServer.h"
#include "StatusUpdater.h"

class BaseStationServer : public SimpleServer
{
  public:
   
    // Constructor
    BaseStationServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater);

  private:

    virtual COMMAND getCommand(String message) override;

    void sendStatus();

    const StatusUpdater& statusUpdater_;

};



#endif
