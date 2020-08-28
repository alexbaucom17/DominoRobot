#ifndef MockSerialComms_h
#define MockSerialComms_h

#include <string>
#include <queue>

#include "SerialCommsBase.h"

class MockSerialComms : public SerialCommsBase
{
  public:
    
    // Constructor
    MockSerialComms(std::string portName);

    virtual ~MockSerialComms();

    void send(std::string msg) override;

    std::string rcv() override;

  protected:

    std::queue<std::string> data_;
    std::string port_;
    

};

#endif