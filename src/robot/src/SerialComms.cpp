#include "SerialComms.h"

SerialComms::SerialComms(std::string portName)
: serial_(portName),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_("")
{
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

SerialComms::~SerialComms()
{
}

std::string SerialComms::rcv()
{
    bool newData = false;
    const int timeout_ms = 50;
    std::string new_msg;
    while (serial_.IsDataAvailable() > 0 && newData == false) 
    {
        char rc = ' ';
        try
        {
            serial_.ReadByte(rc, timeout_ms);
        }
        catch (LibSerial::ReadTimeout&)
        {
            break;
        }

        if (recvInProgress_ == true) 
        {
            if (rc == START_CHAR)
            {
              buffer_ = "";
            }
            else if (rc != END_CHAR) 
            {
                buffer_ += rc;
            }
            else 
            {
                recvInProgress_ = false;
                newData = true;
                new_msg = buffer_;
                buffer_ = "";
            }
        }
        else if (rc == START_CHAR) 
        {
            recvInProgress_ = true;
        }
    }
    return new_msg;
}

void SerialComms::send(std::string msg)
{
    if (msg.length() > 0)
    {
      serial_.WriteByte(START_CHAR);
      serial_.Write(msg);
      serial_.WriteByte(END_CHAR);
    }
}