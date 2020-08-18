#include "SerialComms.h"

#include <plog/Log.h>

std::unique_ptr<SerialCommsBase> buildSerialComms(std::string portName)
{
    try
    {
        return std::make_unique<SerialComms>(portName);
    }
    catch (const LibSerial::OpenFailed&)
    {
        PLOGW << "Could not open serial port: " << portName;
        return std::make_unique<SerialCommsBase>();
    }
}

// TODO: Add system to forward debug messages to the log
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
        PLOGE.printf("Cannot recieve if port isn't connected");
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