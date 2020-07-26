#include "SimpleServer.h"

#include <ArduinoJson/ArduinoJson.h>
#include <plog/Log.h>

#include "SocketWrapper.h"


SimpleServer::SimpleServer()
: clientConnected_(false),
  wifiConnected_(false),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_("")
{
    SocketWrapper s;
}

SimpleServer::~SimpleServer()
{
}

void SimpleServer::begin()
{
}

COMMAND SimpleServer::oneLoop()
{
    COMMAND cmd = COMMAND::NONE;
    std::string newMsg = getAnyIncomingMessage();
    PLOGI.printf("%s",newMsg);
    
    if(newMsg.length() != 0)
    {    
        bool printDebug = true;    
        bool checkCommand = false;
        if(newMsg.find("Client connected") != std::string::npos)
        {
            clientConnected_ = true;
            wifiConnected_ = true;
        }
        else if(newMsg.find("Client disconnected") != std::string::npos)
        {
            clientConnected_ = false;
            wifiConnected_ = true;
        }
        else if(newMsg.find("Connecting..") != std::string::npos)
        {
            wifiConnected_ = false;
            clientConnected_ = false;
        }
        else if(newMsg.find("Connected to WiFi.") != std::string::npos)
        {
            clientConnected_ = false;
            wifiConnected_ = true;
        }
        else if(newMsg.find('*') != std::string::npos)
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
        }

        if(checkCommand)
        {
            cmd = getCommand(cleanString(newMsg));
        }
    }
    return cmd;
}

std::string SimpleServer::cleanString(std::string message)
{
  int idx_start = message.find("{");
  int idx_end = message.find("}") + 1;
  return message.substr(idx_start, idx_end);
}

std::string SimpleServer::getAnyIncomingMessage()
{
    // bool newData = false;
    std::string new_msg = "Test";

    //TODO: Change to use sockets
    // while (serial_.available() > 0 && newData == false) 
    // {
    //     char rc = serial_.read();
    //     //debug_.print(millis());
    //     //debug_.print(" data: ");
    //     //debug_.println(rc);
    //     if (recvInProgress_ == true) 
    //     {
    //         if (rc == START_CHAR)
    //         {
    //           #ifdef PRINT_DEBUG
    //           debug_.println("Receive already in progress! Dropping old message");
    //           debug_.print("Partial message: ");
    //           debug_.println(buffer_);
    //           #endif
    //           buffer_ = "";
    //         }
    //         else if (rc != END_CHAR) 
    //         {
    //             buffer_ += rc;
    //         }
    //         else 
    //         {
    //             recvInProgress_ = false;
    //             newData = true;
    //             new_msg = buffer_;
    //             buffer_ = "";
    //             //debug_.println("Found end char");
    //         }
    //     }
    //     else if (rc == START_CHAR) 
    //     {
    //         recvInProgress_ = true;
    //         //debug_.println("Found start char");
    //     }
    // }
    return new_msg;
}

void SimpleServer::sendMsg(std::string msg, bool print_debug)
{
    if (msg.length() == 0 && print_debug)
    {
      PLOGI.printf("[SimpleServer] Nothing to send!!!\n");
    }
    else
    {
        // TODO: Change to sockets
    //   serial_.print(START_CHAR);
    //   serial_.print(msg);
    //   serial_.print(END_CHAR);
    }
}

void SimpleServer::printIncommingCommand(std::string message)
{
    PLOGI.printf(message.c_str());
}

void SimpleServer::sendAck(std::string data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    std::string msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}

void SimpleServer::sendErr(std::string data)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["data"] = data;
    std::string msg;
    serializeJson(doc, msg);
    sendMsg(msg);
}