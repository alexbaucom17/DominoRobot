#include "SerialComms.h"

#include <plog/Log.h>

SerialComms::SerialComms(std::string portName)
: SerialCommsBase(),
  serial_(portName),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  base_data_(),
  lift_data_(),
  distance_data_()
{
    // If we get here, that means serial_ was constructed correctly which means 
    // we have a valid connection
    connected_ = true;
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);    
}

SerialComms::~SerialComms()
{
}

std::string SerialComms::rcv_base()
{
    rcv();
    if(base_data_.empty())
    {
        return "";
    }
    std::string outdata = base_data_.front();
    base_data_.pop();
    return outdata;
}

std::string SerialComms::rcv_lift()
{
    rcv();
    if(lift_data_.empty())
    {
        return "";
    }
    std::string outdata = lift_data_.front();
    lift_data_.pop();
    return outdata;
}

std::string SerialComms::rcv_distance()
{
    rcv();
    if(distance_data_.empty())
    {
        return "";
    }
    std::string outdata = distance_data_.front();
    distance_data_.pop();
    return outdata;
}

void SerialComms::rcv()
{
    if(!connected_)
    {
        PLOGE.printf("Cannot receive if port isn't connected");
        return;
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

    // Figure out what to do with the message based on the identifier
    if (new_msg.rfind("DEBUG", 0) == 0)
    {
        PLOGI << new_msg;
    }
    else if (new_msg.rfind("base:", 0) == 0)
    {
        base_data_.push(new_msg.substr(5, std::string::npos));
    }
    else if (new_msg.rfind("lift:", 0) == 0)
    {
        lift_data_.push(new_msg.substr(5, std::string::npos));
    }
    else if (new_msg.rfind("dist:", 0) == 0)
    {
        distance_data_.push(new_msg.substr(5, std::string::npos));
    }
    else if (new_msg.empty())
    {
        // Do nothing
    }
    else
    {
        PLOGE << "Unknown message type, skipping: " << new_msg;
    }
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