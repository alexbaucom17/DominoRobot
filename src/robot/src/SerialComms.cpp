#include "SerialComms.h"

#include <plog/Log.h>

SerialComms::SerialComms(std::string portName)
: serial_(),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  connected_(false)
{
    try
    {
        serial_ .Open(portName);
        connected_ = true;
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (LibSerial::OpenFailed&)
    {
        PLOGW.printf("Could not connect to serial port %s", portName.c_str());
    }
    
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
    const int timeout_ms = 50;
    std::string new_msg;
    while (serial_.IsDataAvailable() > 0 && newData == false) 
    {
        char rc;
        try
        {
            // TODO: fix wait - maybe move back to serial port and use a factory class to create serial comms object that can handle the opening exceptions
            serial_ >> rc;
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
    if(!connected_)
    {
        PLOGE.printf("Cannot send if port isn't connected");
        return;
    }
    
    if (msg.length() > 0)
    {
      std::string toSend = START_CHAR + msg + END_CHAR;
      serial_ << toSend;
    }
}