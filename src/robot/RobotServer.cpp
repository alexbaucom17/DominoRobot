
#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include "RobotServer.h"
#include <ArduinoJson.h>

RobotServer::RobotServer(HardwareSerial& serial, HardwareSerial& debug)
: serial_(serial),
  debug_(debug),
  clientConnected_(false),
  wifiConnected_(false),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  moveData_()
{
    serial_.begin(115200);
}

RobotServer::COMMAND RobotServer::oneLoop()
{
    COMMAND cmd = COMMAND::NONE;
    String newMsg = getAnyIncomingMessage();
    
    if(newMsg.length() != 0)
    {    
        bool printDebug = true;    
        bool checkCommand = false;
        if(newMsg.lastIndexOf("Client connected") >= 0)
        {
            clientConnected_ = true;
            wifiConnected_ = true;
        }
        else if(newMsg.lastIndexOf("Client disconnected") >= 0)
        {
            clientConnected_ = false;
            wifiConnected_ = true;
        }
        else if(newMsg.lastIndexOf("Connecting..") >= 0)
        {
            wifiConnected_ = false;
            clientConnected_ = false;
        }
        else if(newMsg.lastIndexOf("Connected to WiFi.") >= 0)
        {
            clientConnected_ = false;
            wifiConnected_ = true;
        }
        else if(newMsg.lastIndexOf('*') >= 0)
        {
            clientConnected_ = false;
            wifiConnected_ = true;
            printDebug = false;
        }
        else
        {
            checkCommand = true;
            printDebug = true;
        }

        if(printDebug)
        {
            debug_.print("[RobotServer] ");
            debug_.print("RCV: ");
            debug_.println(newMsg);
        }

        if(checkCommand)
        {
            cmd = getCommand(cleanString(newMsg));
        }
    }
    return cmd;
}

String RobotServer::cleanString(String message)
{
  int idx_start = message.indexOf("{");
  int idx_end = message.lastIndexOf("}") + 1;
  return message.substring(idx_start, idx_end);
}

String RobotServer::getAnyIncomingMessage()
{
    bool newData = false;
    String new_msg;
    while (serial_.available() > 0 && newData == false) 
    {
        char rc = serial_.read();
        //debug_.print("data: ");
        //debug_.println(rc);
        if (recvInProgress_ == true) 
        {
            if (rc != END_CHAR) 
            {
                buffer_ += rc;
            }
            else 
            {
                recvInProgress_ = false;
                newData = true;
                new_msg = buffer_;
                buffer_ = "";
                //debug_.println("Found end char");
            }
        }
        else if (rc == START_CHAR) 
        {
            recvInProgress_ = true;
            //debug_.println("Found start char");
        }
    }
    return new_msg;
}

RobotServer::COMMAND RobotServer::getCommand(String message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    debug_.print("[RobotServer] GetCommand(): ");
    debug_.println(message);

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
            debug_.println("[RobotServer] Got MOVE command ");
            cmd = COMMAND::MOVE;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            sendAck(type);
        }
        else if(type == "place")
        {
            debug_.println("[RobotServer] Got PLACE command ");
            cmd = COMMAND::PLACE;
            sendAck(type);
        }
        else if(type == "dock")
        {
            debug_.println("[RobotServer] Got DOCK command ");
            cmd = COMMAND::DOCK;
            sendAck(type);
        }
        else if(type == "undock")
        {
            debug_.println("[RobotServer] Got UNDOCK command ");
            cmd = COMMAND::UNDOCK;
            sendAck(type);
        }
        else if(type == "dropoff")
        {
            debug_.println("[RobotServer] Got DROPOFF command ");
            cmd = COMMAND::DROPOFF;
            sendAck(type);
        }
        else if(type == "pickup")
        {
            debug_.println("[RobotServer] Got PICKUP command ");
            cmd = COMMAND::PICKUP;
            sendAck(type);
        }
        else if(type == "position")
        {
            debug_.println("[RobotServer] Got POSITION command ");
            cmd = COMMAND::POSITION;
            sendAck(type);
        }
        else if(type == "status")
        {
            debug_.println("[RobotServer] Got STATUS command ");
            cmd = COMMAND::STATUS;
            sendStatus();
        }
        else if(type == "")
        {
            debug_.println("[RobotServer] ERROR: Type field empty or not specified ");
            cmd = COMMAND::ERROR_NO_TYPE;
            sendErr("no_type");
        }
        else
        {
            debug_.println("[RobotServer] ERROR: Unkown type field ");
            cmd = COMMAND::ERROR_UNKOWN_TYPE;
            sendErr("unkown_type");
        }
    }
    return cmd;    
}

RobotServer::MoveData RobotServer::getMoveData()
{
    return moveData_;
}

void RobotServer::sendMsg(String msg)
{
    serial_.print(START_CHAR);
    serial_.print(msg);
    serial_.print(END_CHAR);
    debug_.print("[RobotServer] Send: ");
    debug_.println(msg);
}

void RobotServer::sendAck(String data)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    String msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void RobotServer::sendErr(String data)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    String msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void RobotServer::sendStatus()
{
    StaticJsonDocument<256> doc;
    doc["type"] = "status";
    doc["data"] = "not implimented";
    String msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}