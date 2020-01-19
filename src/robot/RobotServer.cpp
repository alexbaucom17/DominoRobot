
#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include "RobotServer.h"
#include <ArduinoJson.h>

RobotServer::RobotServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater)
: serial_(serial),
  debug_(debug),
  clientConnected_(false),
  wifiConnected_(false),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  moveData_(),
  statusUpdater_(statusUpdater)
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
            #ifdef PRINT_DEBUG
            debug_.print("[RobotServer] ");
            debug_.print("RX: ");
            debug_.println(newMsg);
            #endif
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
        //debug_.print(millis());
        //debug_.print(" data: ");
        //debug_.println(rc);
        if (recvInProgress_ == true) 
        {
            if (rc == START_CHAR)
            {
              #ifdef PRINT_DEBUG
              debug_.println("Receive already in progress! Dropping old message");
              debug_.print("Partial message: ");
              debug_.println(buffer_);
              #endif
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

    //debug_.print("[RobotServer] GetCommand(): ");
    //debug_.println(message);

    if(err)
    {
        #ifdef PRINT_DEBUG
        debug_.print("[RobotServer] Error parsing JSON: ");
        debug_.println(err.c_str());   
        #endif
        cmd = COMMAND::ERROR_BAD_JSON;
        sendErr("bad_json");
    }
    else
    {
        String type = doc["type"];
        if(type == "move")
        {
            cmd = COMMAND::MOVE;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            sendAck(type);
        }
        else if(type == "move_rel")
        {
            cmd = COMMAND::MOVE_REL;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            sendAck(type);
        }
        else if(type == "move_fine")
        {
            cmd = COMMAND::MOVE_FINE;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            sendAck(type);
        }
        else if(type == "place")
        {
            cmd = COMMAND::PLACE;
            sendAck(type);
        }
        else if(type == "p")
        {
            cmd = COMMAND::POSITION;
            positionData_.x = doc["data"]["x"];
            positionData_.y = doc["data"]["y"];
            positionData_.a = doc["data"]["a"];
            sendAck(type);
        }
        else if(type == "status")
        {
            cmd = COMMAND::STATUS;
            sendStatus();
        }
        else if (type == "check")
        {
            cmd = COMMAND::CHECK;
            sendAck(type);
        }
        else if(type == "")
        {
            #ifdef PRINT_DEBUG
            debug_.println("[RobotServer] ERROR: Type field empty or not specified ");
            #endif
            cmd = COMMAND::ERROR_NO_TYPE;
            sendErr("no_type");
        }
        else
        {
            #ifdef PRINT_DEBUG
            debug_.println("[RobotServer] ERROR: Unkown type field ");
            #endif
            cmd = COMMAND::ERROR_UNKOWN_TYPE;
            sendErr("unkown_type");
        }
    }
    return cmd;    
}

RobotServer::PositionData RobotServer::getMoveData()
{
    return moveData_;
}

RobotServer::PositionData RobotServer::getPositionData()
{
    return positionData_;
}

void RobotServer::sendMsg(String msg)
{
    if (msg.length() == 0)
    {
      debug_.println("[RobotServer] Nothing to send!!!");
    }
    else
    {
      serial_.print(START_CHAR);
      serial_.print(msg);
      serial_.print(END_CHAR);
      #ifdef PRINT_DEBUG
      debug_.print("[RobotServer] TX: ");
      debug_.println(msg);
      #endif
    }
}

void RobotServer::sendAck(String data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    String msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void RobotServer::sendErr(String data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    String msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void RobotServer::sendStatus()
{
    String msg = statusUpdater_.getStatusJsonString();
    sendMsg(msg);
}
