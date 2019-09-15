#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include "RobotServer.h"
#include <ArduinoJson.h>


RobotServer::RobotServer(HardwareSerial& serial, HardwareSerial& debug)
: serial_(serial),
  debug_(debug),
  clientConnected_(false)
{
    serial_.begin(115200);
}

RobotServer::oneLoop()
{
    String newMsg = getAnyIncomingMessage();
    COMMAND cmd = COMMAND::NONE;
    if(newMsg.length() != 0)
    {
        debug_.print("[RobotServer] ");
        debug_.println(newMsg);

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
            cmd = getCommand(newMsg);
        }
    }

    return cmd;
}

String RobotServer::getAnyIncomingMessage()
{
    String msg = "";
    if(serial_.available())
    {
        msg = serial_.readString();
    }
    return msg;
}

RobotServer::COMMAND RobotServer::getCommand(String message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    if(err)
    {
        debug_.print("[RobotServer] Error parsing JSON: ");
        debug_.println(err.c_str());   
        cmd = COMMAND::ERROR_BAD_JSON;
        sendErr("bad_json");
    }
    else
    {
        String type = doc["type"];
        if(type == "move")
        {
            debug_.print("[RobotServer] Got MOVE command ");
            cmd = COMMAND::MOVE;
            sendAck(type);
        }
        else if(type == "place")
        {
            debug_.print("[RobotServer] Got PLACE command ");
            cmd = COMMAND::PLACE;
            sendAck(type);
        }
        else if(type == "dock")
        {
            debug_.print("[RobotServer] Got DOCK command ");
            cmd = COMMAND::DOCK;
            sendAck(type);
        }
        else if(type == "undock")
        {
            debug_.print("[RobotServer] Got UNDOCK command ");
            cmd = COMMAND::UNDOCK;
            sendAck(type);
        }
        else if(type == "dropoff")
        {
            debug_.print("[RobotServer] Got DROPOFF command ");
            cmd = COMMAND::DROPOFF;
            sendAck(type);
        }
        else if(type == "pickup")
        {
            debug_.print("[RobotServer] Got PICKUP command ");
            cmd = COMMAND::PICKUP;
            sendAck(type);
        }
        else if(type == "position")
        {
            debug_.print("[RobotServer] Got POSITION command ");
            cmd = COMMAND::POSITION;
            sendAck(type);
        }
        else if(type == "status")
        {
            debug_.print("[RobotServer] Got STATUS command ");
            cmd = COMMAND::STATUS;
            sendStatus();
        }
        else if(type == "")
        {
            debug_.print("[RobotServer] ERROR: Type field empty or not specified ");
            cmd = COMMAND::ERROR_NO_TYPE;
            sendErr("no_type");
        }
        else
        {
            debug_.print("[RobotServer] ERROR: Unkown type field ");
            cmd = COMMAND::ERROR_UNKOWN_TYPE;
            sendErr("unkown_type");
        }
    }
    return cmd;    
}

void RobotServer::sendAck(String data)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    serializeJson(doc, serial_);
}

void RobotServer::sendErr(String data)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "error";
    doc["data"] = data;
    serializeJson(doc, serial_);
}

void RobotServer::sendStatus()
{
    StaticJsonDocument<256> doc;
    doc["type"] = "status";
    doc["data"] = "not implimented";
    serializeJson(doc, serial_);
}
