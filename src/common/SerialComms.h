#ifndef SerialComms_h
#define SerialComms_h

#include <HardwareSerial.h>

#define START_CHAR '<'
#define END_CHAR '>'

class SerialComms
{
  public:
    
    // Constructor
    SerialComms(HardwareSerial& serial, HardwareSerial& debug);

    virtual ~SerialComms();

    void send(String msg);

    String rcv();

  protected:

    HardwareSerial& serial_;
    HardwareSerial& debug_;

    bool recvInProgress_;
    int recvIdx_;
    String buffer_;

}

#endif