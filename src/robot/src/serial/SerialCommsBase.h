#ifndef SerialCommsBase_h
#define SerialCommsBase_h

#include <string>

#define START_CHAR '<'
#define END_CHAR '>'

class SerialCommsBase
{
  public:

    SerialCommsBase();

    virtual ~SerialCommsBase();

    virtual void send(std::string msg);

    virtual std::string rcv_base();

    virtual std::string rcv_lift();

    virtual std::string rcv_distance();

    bool isConnected() {return connected_;};

  protected:

    bool connected_;

};

#endif