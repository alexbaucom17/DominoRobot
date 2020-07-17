#include "SerialComms.h"

SerialComms::SerialComms(HardwareSerial& serial, HardwareSerial& debug)
: serial_(serial),
  debug_(debug),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_("")
{
    serial_.begin(115200);
}

SerialComms::~SerialComms()
{
}


String SerialComms::rcv()
{
    bool newData = false;
    String new_msg;
    while (serial_.available() > 0 && newData == false) 
    {
        char rc = serial_.read();
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

void SerialComms::send(String msg)
{
    if (msg.length() > 0)
    {
      serial_.print(START_CHAR);
      serial_.print(msg);
      serial_.print(END_CHAR);
    }
}