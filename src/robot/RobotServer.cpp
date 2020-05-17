
#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include "RobotServer.h"
#include <ArduinoJson.h>
#include "constants.h"  // FOR PRINT_DEBUG

RobotServer::RobotServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater)
: SimpleServer(serial, debug),
  moveData_(),
  positionData_(),
  statusUpdater_(statusUpdater)
{
}

COMMAND RobotServer::getCommand(String message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    if(err)
    {
        #ifdef PRINT_DEBUG
        printIncommingCommand(message);
        debug_.print("[RobotServer] Error parsing JSON: ");
        debug_.println(err.c_str());   
        #endif
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
            #ifdef PRINT_DEBUG
            printIncommingCommand(message);
            debug_.println("[RobotServer] ERROR: Type field empty or not specified ");
            #endif
            sendErr("no_type");
        }
        else
        {
            #ifdef PRINT_DEBUG
            printIncommingCommand(message);
            debug_.println("[RobotServer] ERROR: Unkown type field ");
            #endif
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

void RobotServer::sendStatus()
{
    String msg = statusUpdater_.getStatusJsonString();
    sendMsg(msg, false);
}
