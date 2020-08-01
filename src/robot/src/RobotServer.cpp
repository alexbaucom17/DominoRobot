
#include "RobotServer.h"

#include <ArduinoJson/ArduinoJson.h>
#include <plog/Log.h>

#include "sockets/SocketMultiThreadWrapperFactory.h"

RobotServer::RobotServer(StatusUpdater& statusUpdater)
: moveData_(),
  positionData_(),
  velocityData_(),
  statusUpdater_(statusUpdater),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  socket_(SocketMultiThreadWrapperFactory::getFactoryInstance()->get_socket())
{
}

COMMAND RobotServer::getCommand(std::string message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    if(err)
    {
        printIncommingCommand(message);
        PLOGI.printf("Error parsing JSON: ");
        PLOGI.printf(err.c_str());   
        sendErr("bad_json");
    }
    else
    {
        std::string type = doc["type"];
        if(type == "move")
        {
            cmd = COMMAND::MOVE;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "move_rel")
        {
            cmd = COMMAND::MOVE_REL;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "move_fine")
        {
            cmd = COMMAND::MOVE_FINE;
            moveData_.x = doc["data"]["x"];
            moveData_.y = doc["data"]["y"];
            moveData_.a = doc["data"]["a"];
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "move_const_vel")
        {
            cmd = COMMAND::MOVE_CONST_VEL;
            velocityData_.vx = doc["data"]["vx"];
            velocityData_.vy = doc["data"]["vy"];
            velocityData_.va = doc["data"]["va"];
            velocityData_.t = doc["data"]["t"];
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "place")
        {
            cmd = COMMAND::PLACE_TRAY;
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "load")
        {
            cmd = COMMAND::LOAD_TRAY;
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "init")
        {
            cmd = COMMAND::INITIALIZE_TRAY;
            printIncommingCommand(message);
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
        else if(type == "estop")
        {
            cmd = COMMAND::ESTOP;
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "lc")
        {
            cmd = COMMAND::LOAD_COMPLETE;
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "status")
        {
            sendStatus();
        }
        else if (type == "check")
        {
            sendAck(type);
        }
        else if(type == "")
        {
            printIncommingCommand(message);
            PLOGI.printf("ERROR: Type field empty or not specified ");
            sendErr("no_type");
        }
        else
        {
            printIncommingCommand(message);
            PLOGI.printf("ERROR: Unkown type field ");
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

RobotServer::VelocityData RobotServer::getVelocityData()
{
    return velocityData_;
}

void RobotServer::sendStatus()
{
    std::string msg = statusUpdater_.getStatusJsonString();
    sendMsg(msg, false);
}

COMMAND RobotServer::oneLoop()
{
    COMMAND cmd = COMMAND::NONE;
    std::string newMsg = getAnyIncomingMessage();
    
    if(newMsg.length() != 0)
    {    
        PLOGD.printf("RX: %s", newMsg.c_str());
        cmd = getCommand(cleanString(newMsg));
    }
    return cmd;
}

std::string RobotServer::cleanString(std::string message)
{
  int idx_start = message.find("{");
  int idx_end = message.find("}") + 1;
  int len = idx_end - idx_start + 1;
  if(idx_start == -1 || idx_end == 0)
  {
      PLOGW.printf("Could not find brackets in message");
      return message;
  }
  return message.substr(idx_start, len);
}

std::string RobotServer::getAnyIncomingMessage()
{
    bool newData = false;
    std::string new_msg = "";

    while (socket_->dataAvailableToRead() && newData == false) 
    {
        std::string data = socket_->getData();
        for (auto c : data)
        {
            if (recvInProgress_ == true) 
            {
                if (c == START_CHAR)
                {
                    buffer_ = "";
                }
                else if (c != END_CHAR) 
                {
                    buffer_ += c;
                }
                else 
                {
                    recvInProgress_ = false;
                    newData = true;
                    new_msg = buffer_;
                    buffer_ = "";
                }
            }
            else if (c == START_CHAR) 
            {
                recvInProgress_ = true;
            }
        }
    }
    return new_msg;
}

void RobotServer::sendMsg(std::string msg, bool print_debug)
{
    if (msg.length() == 0 && print_debug)
    {
      PLOGI.printf("Nothing to send!!!\n");
    }
    else
    {
        if(print_debug)
        {
            PLOGD.printf("TX: %s", msg.c_str());
        }

        std::string send_msg = START_CHAR + msg + END_CHAR;
        socket_->sendData(send_msg);
    }
}

void RobotServer::printIncommingCommand(std::string message)
{
    PLOGI.printf(message.c_str());
}

void RobotServer::sendAck(std::string data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    std::string msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void RobotServer::sendErr(std::string data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    std::string msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}