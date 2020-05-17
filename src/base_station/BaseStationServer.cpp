
#include <Arduino.h> // This has to be before ArduinoJson.h to fix compiler issues
#include "BaseStationServer.h"
#include <ArduinoJson.h>
#include "constants.h"  // FOR PRINT_DEBUG

BaseStationServer::BaseStationServer(HardwareSerial& serial, HardwareSerial& debug, const StatusUpdater& statusUpdater)
: SimpleServer(serial, debug),
  statusUpdater_(statusUpdater)
{
}

COMMAND BaseStationServer::getCommand(String message)
{
    COMMAND cmd = COMMAND::NONE;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);

    if(err)
    {
        #ifdef PRINT_DEBUG
        printIncommingCommand(message);
        debug_.print("[BaseStationServer] Error parsing JSON: ");
        debug_.println(err.c_str());   
        #endif
        sendErr("bad_json");
    }
    else
    {
        String type = doc["type"];
        if(type == "estop")
        {
            cmd = COMMAND::ESTOP;
            printIncommingCommand(message);
            sendAck(type);
        }
        else if(type == "")
        {
            #ifdef PRINT_DEBUG
            printIncommingCommand(message);
            debug_.println("[BaseStationServer] ERROR: Type field empty or not specified ");
            #endif
            sendErr("no_type");
        }
        else
        {
            #ifdef PRINT_DEBUG
            printIncommingCommand(message);
            debug_.println("[BaseStationServer] ERROR: Unkown type field ");
            #endif
            sendErr("unkown_type");
        }
    }
    return cmd;    
}

void BaseStationServer::sendStatus()
{
    String msg = statusUpdater_.getStatusJsonString();
    sendMsg(msg, false);
}
