#include "SimpleServer.h"

#include <ArduinoJson/ArduinoJson.h>
#include <plog/Log.h>



SimpleServer::SimpleServer()
: clientConnected_(false),
  wifiConnected_(false),
  recvInProgress_(false),
  recvIdx_(0),
  buffer_(""),
  socket_()
{
}

SimpleServer::~SimpleServer()
{
}


COMMAND SimpleServer::oneLoop()
{
    COMMAND cmd = COMMAND::NONE;
    std::string newMsg = getAnyIncomingMessage();
    
    if(newMsg.length() != 0)
    {    
        cmd = getCommand(cleanString(newMsg));
    }
    return cmd;
}

std::string SimpleServer::cleanString(std::string message)
{
  int idx_start = message.find("{");
  int idx_end = message.find("}") + 1;
  int len = idx_end - idx_start;
  if(idx_start == -1 || idx_end == 0)
  {
      PLOGW.printf("Could not find brackets in message");
      return message;
  }
  return message.substr(idx_start, len);
}

std::string SimpleServer::getAnyIncomingMessage()
{
    bool newData = false;
    std::string new_msg = "";

    while (socket_.dataAvailableToRead() && newData == false) 
    {
        std::string data = socket_.getData();
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

void SimpleServer::sendMsg(std::string msg, bool print_debug)
{
    if (msg.length() == 0 && print_debug)
    {
      PLOGI.printf("[SimpleServer] Nothing to send!!!\n");
    }
    else
    {
        std::string send_msg = START_CHAR + msg + END_CHAR;
        socket_.sendData(send_msg);
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