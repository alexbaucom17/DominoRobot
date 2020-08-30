#include "SerialComms.h"

#include <plog/Log.h>

SerialComms::SerialComms(std::string portName)
: SerialCommsBase(),
  serial_(portName),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_("")
{
    // If we get here, that means serial_ was constructed correctly which means 
    // we have a valid connection
    connected_ = true;
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);    
}

SerialComms::~SerialComms()
{
}

std::string SerialComms::rcv()
{
    if(!connected_)
    {
        PLOGE.printf("Cannot receive if port isn't connected");
        return "";
    }
    
    bool newData = false;
    const int timeout_ms = 5;
    std::string new_msg;
    while (serial_.IsDataAvailable() && newData == false) 
    {
        char rc = ' ';
        try
        {
            serial_.ReadByte(rc, timeout_ms);
        }
        catch (LibSerial::ReadTimeout&)
        {
            PLOGI.printf("Serial timeout");
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

    // If the client is sending a debug message, just log it
    if (new_msg.rfind("DEBUG", 0) == 0)
    {
        PLOGI << new_msg;
        return "";
    }

    return new_msg;
}

void SerialComms::send(std::string msg)
{
    if(!connected_)
    {
        PLOGE.printf("Cannot send if port isn't connected");
        return;
    }
    
    if (msg.length() > 0)
    {
      std::string toSend = START_CHAR + msg + END_CHAR;
      PLOGD.printf("Serial send: %s",toSend.c_str());
      serial_.Write(toSend);
    }
}