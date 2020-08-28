#ifndef SerialCommsFactory_h
#define SerialCommsFactory_h

#include <memory>

#include "SerialCommsBase.h"

enum SERIAL_FACTORY_MODE
{
    STANDARD = 0,
    MOCK = 1,
};

class SerialCommsFactory
{

  public:

    static SerialCommsFactory* getFactoryInstance();

    void set_mode(SERIAL_FACTORY_MODE mode);

    std::unique_ptr<SerialCommsBase> build_serial_comms(std::string portName);

    // Delete copy and assignment constructors
    SerialCommsFactory(SerialCommsFactory const&) = delete;
    SerialCommsFactory& operator= (SerialCommsFactory const&) = delete;

  private:

    // Make standard constructor private so it can't be created
    SerialCommsFactory();

    static SerialCommsFactory* instance;

    SERIAL_FACTORY_MODE mode_;

};


#endif