#include "RobotServer.h"
#include <ArduinoJson.h>

RobotServer::RobotServer(HardwareSerial serial)
: serial_(serial),
  debug_(debug),
  clientConnected_(false)
{
    serial_.begin(115200);
}

RobotServer::oneLoop()
{
    string newMsg = getAnyIncomingMessage();
    COMMAND returnCmd = COMMAND::NONE;
    if(newMsg.length() != 0)
    {
        debug_.write("[RobotServer] ");
        debug_.writeln(newMsg);

        if(newMsg == "Client connected")
        {
            clientConnected_ = true;
        }
        else if(newMsg == "Client disconnected")
        {
            clientConnected_ = false;
        }
        else
        {
            returnCmd = handleCommand(newMsg);
        }
    }

    return returnCmd;
    
}

string RobotServer::getAnyIncomingMessage()
{
    string msg = "";
    if(serial_.available())
    {
        msg = serial_.readString();
    }
    return msg;
}

void RobotServer::sendMessage(string message)
{
    serial_.write(message);
}

RobotServer::COMMAND RobotServer::handleCommand(string message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    if(err)
    {
        debug_.write("[RobotServer] Error parsing JSON: ");
        debug_.writeln(err.c_str());   
    }
    else
    {
        string type = doc["type"];
        switch(type)
        {
            case "move":
            case "place":
            case "dock":
            case "undock":
            case "dropoff":
            case "pickup":
            case "position":
            case "status":
            case "":
                debug_.write("[RobotServer] ERROR: Type field empty or not specified ");
                break;
            default:
                debug_.write("[RobotServer] ERROR: Unkown type field ");

        }
    }
    
}