#ifndef SerialCommsFactory_h
#define SerialCommsFactory_h

#include <memory>
#include <map>

#include "SerialCommsBase.h"

enum class SERIAL_FACTORY_MODE
{
  STANDARD,
  MOCK,
};

class SerialCommsFactory
{

  public:

    static SerialCommsFactory* getFactoryInstance();

    void set_mode(SERIAL_FACTORY_MODE mode);

    SerialCommsBase* get_serial_comms(std::string portName);

    // Delete copy and assignment constructors
    SerialCommsFactory(SerialCommsFactory const&) = delete;
    SerialCommsFactory& operator= (SerialCommsFactory const&) = delete;

  private:

    // Make standard constructor private so it can't be created
    SerialCommsFactory();

    void build_serial_comms(std::string portName);

    static SerialCommsFactory* instance;

    SERIAL_FACTORY_MODE mode_;

    std::map<std::string, std::unique_ptr<SerialCommsBase>> comms_objects_;

};


#endif