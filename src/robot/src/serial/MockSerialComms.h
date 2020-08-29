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

    void mock_send(std::string msg);

    std::string mock_rcv();

    void purge_data();

  protected:

    std::queue<std::string> send_data_;
    std::queue<std::string> rcv_data_;
    std::string port_;
    

};

#endif