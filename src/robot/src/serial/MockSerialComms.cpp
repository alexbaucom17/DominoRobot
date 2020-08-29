#include "MockSerialComms.h"


MockSerialComms::MockSerialComms(std::string portName)
: SerialCommsBase(),
  send_data_(),
  rcv_data_(),
  port_(portName)
{
    connected_ = true;
}

MockSerialComms::~MockSerialComms()
{
}

void MockSerialComms::send(std::string msg)
{
    send_data_.push(msg);
}

std::string MockSerialComms::rcv()
{
    if(rcv_data_.empty())
    {
        return "";
    }
    std::string outdata = rcv_data_.front();
    rcv_data_.pop();
    return outdata;
}

void MockSerialComms::mock_send(std::string msg)
{
    rcv_data_.push(msg);
}

std::string MockSerialComms::mock_rcv()
{
    if(send_data_.empty())
    {
        return "";
    }
    std::string outdata = send_data_.front();
    send_data_.pop();
    return outdata;
}

void MockSerialComms::purge_data()
{
    while(!send_data_.empty())
    {
        send_data_.pop();
    }
    while(!rcv_data_.empty())
    {
        rcv_data_.pop();
    }
}