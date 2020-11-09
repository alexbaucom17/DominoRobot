#ifndef testutils_h
#define testutils_h

#include "serial/MockSerialComms.h"
#include "serial/SerialCommsFactory.h"
#include "constants.h"
#include "sockets/SocketMultiThreadWrapperFactory.h"
#include "sockets/MockSocketMultiThreadWrapper.h"

inline MockSocketMultiThreadWrapper* build_and_get_mock_socket() 
{
    SocketMultiThreadWrapperBase* base_socket = SocketMultiThreadWrapperFactory::getFactoryInstance()->get_socket();
    // Slightly dangerous....
    MockSocketMultiThreadWrapper* mock_socket = dynamic_cast<MockSocketMultiThreadWrapper*>(base_socket);
    mock_socket->purge_data();
    return mock_socket;
}

inline MockSerialComms* build_and_get_mock_serial()
{
    SerialCommsBase* base_serial = SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB);
    // Slightly dangerous....
    MockSerialComms* mock_serial = dynamic_cast<MockSerialComms*>(base_serial);
    mock_serial->purge_data();
    return mock_serial;
}

#endif