#include "MockSerialComms.h"


MockSerialComms::MockSerialComms(std::string portName)
: SerialCommsBase(),
  data_(),
  port_(portName)
{
    connected_ = true;
}

MockSerialComms::~MockSerialComms()
{
}

void MockSerialComms::send(std::string msg)
{
    data_.push(msg);
}

std::string MockSerialComms::rcv()
{
    std::string outdata = data_.front();
    data_.pop();
    return outdata;
}