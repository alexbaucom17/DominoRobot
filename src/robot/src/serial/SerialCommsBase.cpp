#include "SerialCommsBase.h"
#include <plog/Log.h>

SerialCommsBase::SerialCommsBase() : connected_(false) {}

SerialCommsBase::~SerialCommsBase() {}

std::string SerialCommsBase::rcv()
{
    PLOGW.printf("Cannot recieve if port isn't connected");
    return "";
} 

void SerialCommsBase::send(std::string msg)
{
    (void) msg; // Silence warnings
    PLOGW.printf("Cannot send if port isn't connected");
    return;
}