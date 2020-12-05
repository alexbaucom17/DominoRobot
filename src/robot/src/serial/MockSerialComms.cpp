#include "MockSerialComms.h"

#include <plog/Log.h> 


MockSerialComms::MockSerialComms(std::string portName)
: SerialCommsBase(),
  send_base_data_(),
  send_lift_data_(),
  rcv_base_data_(),
  rcv_lift_data_(),
  port_(portName)
{
    connected_ = true;
}

MockSerialComms::~MockSerialComms()
{
}

void MockSerialComms::send(std::string msg)
{
    if (msg.rfind("base:", 0) == 0)
    {
        send_base_data_.push(msg.substr(5, std::string::npos));
    }
    else if (msg.rfind("lift:", 0) == 0)
    {
        send_lift_data_.push(msg.substr(5, std::string::npos));
    }
    else
    {
        PLOGE << "Unknown message type, skipping: " << msg;
    }
}

std::string MockSerialComms::rcv_base()
{
    if(rcv_base_data_.empty())
    {
        return "";
    }
    std::string outdata = rcv_base_data_.front();
    rcv_base_data_.pop();
    return outdata;
}

std::string MockSerialComms::rcv_lift()
{
    if(rcv_lift_data_.empty())
    {
        return "";
    }
    std::string outdata = rcv_lift_data_.front();
    rcv_lift_data_.pop();
    return outdata;
}

void MockSerialComms::mock_send(std::string msg)
{
    if (msg.rfind("base:", 0) == 0)
    {
        rcv_base_data_.push(msg.substr(5, std::string::npos));
    }
    else if (msg.rfind("lift:", 0) == 0)
    {
        rcv_lift_data_.push(msg.substr(5, std::string::npos));
    }
    else
    {
        PLOGE << "Unknown message type, skipping: " << msg;
    }
}

std::string MockSerialComms::mock_rcv_base()
{
    if(send_base_data_.empty())
    {
        return "";
    }
    std::string outdata = send_base_data_.front();
    send_base_data_.pop();
    return outdata;
}

std::string MockSerialComms::mock_rcv_lift()
{
    if(send_lift_data_.empty())
    {
        return "";
    }
    std::string outdata = send_lift_data_.front();
    send_lift_data_.pop();
    return outdata;
}

void MockSerialComms::purge_data()
{
    while(!send_lift_data_.empty())
    {
        send_lift_data_.pop();
    }
    while(!send_base_data_.empty())
    {
        send_base_data_.pop();
    }
    while(!rcv_base_data_.empty())
    {
        rcv_base_data_.pop();
    }
    while(!rcv_lift_data_.empty())
    {
        rcv_lift_data_.pop();
    }
}