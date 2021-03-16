#include "SerialCommsFactory.h"
#include "SerialComms.h"
#include "MockSerialComms.h"

#include <plog/Log.h> 

SerialCommsFactory* SerialCommsFactory::instance = NULL;


SerialCommsFactory* SerialCommsFactory::getFactoryInstance()
{
    if(!instance)
    {
        instance = new SerialCommsFactory;
    }
    return instance;
}

void SerialCommsFactory::set_mode(SERIAL_FACTORY_MODE mode)
{
    mode_ = mode;
}

SerialCommsBase* SerialCommsFactory::get_serial_comms(std::string portName)
{
    if (comms_objects_.count(portName) == 0)
    {
        build_serial_comms(portName);
    }
    return comms_objects_[portName].get();
}


void SerialCommsFactory::build_serial_comms(std::string portName)
{
    std::unique_ptr<SerialCommsBase> serial_comms;
    
    if(mode_ == SERIAL_FACTORY_MODE::STANDARD)
    {
        try
        {
            serial_comms = std::make_unique<SerialComms>(portName);
            PLOGI << "Built SerialComms on port: "<< portName;
        }
        catch (const LibSerial::OpenFailed&)
        {
            serial_comms = std::make_unique<SerialCommsBase>();
            PLOGW << "Could not open serial port: " << portName;
        }

    }
    else if (mode_ == SERIAL_FACTORY_MODE::MOCK)
    {
        serial_comms = std::make_unique<MockSerialComms>(portName);
        PLOGI << "Built MockSerialComms for " << portName;
    }

    comms_objects_[portName] = std::move(serial_comms);
}


// Private constructor
SerialCommsFactory::SerialCommsFactory()
: mode_(SERIAL_FACTORY_MODE::STANDARD),
  comms_objects_()
{}
  