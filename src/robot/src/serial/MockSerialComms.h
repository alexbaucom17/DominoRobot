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

    std::string rcv_base() override;

    std::string rcv_lift() override;

    std::string rcv_distance() override;

    void mock_send(std::string msg);
    
    std::string mock_rcv_lift();
    
    std::string mock_rcv_base();

    std::string mock_rcv_distance();

    void purge_data();

  protected:

    std::queue<std::string> send_base_data_;
    std::queue<std::string> send_lift_data_;
    std::queue<std::string> send_distance_data_;
    std::queue<std::string> rcv_base_data_;
    std::queue<std::string> rcv_lift_data_;
    std::queue<std::string> rcv_distance_data_;
    std::string port_;
    

};

#endif