#ifndef SerialComms_h
#define SerialComms_h

#include <libserial/SerialPort.h>

#include "SerialCommsBase.h"

// Factory method
std::unique_ptr<SerialCommsBase> buildSerialComms(std::string portName);

class SerialComms : public SerialCommsBase
{
  public:
    
    // Constructor
    SerialComms(std::string portName);

    virtual ~SerialComms();

    void send(std::string msg) override;

    std::string rcv() override;


  protected:

    LibSerial::SerialPort serial_;;

    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;
    

};

#endif