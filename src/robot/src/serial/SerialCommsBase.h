#ifndef SerialCommsBase_h
#define SerialCommsBase_h

#include <string>

class SerialCommsBase
{
  public:

    SerialCommsBase();

    virtual ~SerialCommsBase();

    virtual void send(std::string msg);

    virtual std::string rcv();

    bool isConnected() {return connected_;};

  protected:

    bool connected_;

};

#endif