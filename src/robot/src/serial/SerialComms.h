#ifndef SerialComms_h
#define SerialComms_h

#include <libserial/SerialPort.h>
#include <queue>

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

    std::string rcv_base() override;

    std::string rcv_lift() override;

  protected:

    void rcv();

    LibSerial::SerialPort serial_;;

    bool recvInProgress_;
    int recvIdx_;
    std::string buffer_;
    
    std::queue<std::string> base_data_;
    std::queue<std::string> lift_data_;
    

};

#endif