#include "SerialCommsBase.h"
#include <plog/Log.h>

SerialCommsBase::SerialCommsBase() : connected_(false) {}

SerialCommsBase::~SerialCommsBase() {}

std::string SerialCommsBase::rcv_base()
{
    return "";
} 

std::string SerialCommsBase::rcv_lift()
{
    return "";
} 

std::string SerialCommsBase::rcv_distance()
{
    return "";
} 

void SerialCommsBase::send(std::string msg)
{
    (void) msg; // Silence warnings
    return;
}