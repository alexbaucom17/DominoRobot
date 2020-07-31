#ifndef SerialComms_h
#define SerialComms_h

#include <libserial/SerialPort.h>

#define START_CHAR '<'
#define END_CHAR '>'

class SerialComms
{
  public:
    
    // Constructor
    SerialComms(std::string portName);

    virtual ~SerialComms();

    void send(std::string msg);

    std::string rcv();

    bool isConnected() {return connected_;}

  protected:

    LibSerial::SerialPort serial_;;

    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;
    bool connected_;

};

#endif