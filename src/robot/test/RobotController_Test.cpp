#include <Catch/catch.hpp>

#include "RobotController.h"
#include "StatusUpdater.h"
#include "serial/MockSerialComms.h"
#include "serial/SerialCommsFactory.h"
#include "constants.h"

// TODO: Add more tests

MockSerialComms* build_and_get_mock_serial()
{
    SerialCommsBase* base_serial = SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB);
    // Slightly dangerous....
    MockSerialComms* mock_serial = dynamic_cast<MockSerialComms*>(base_serial);
    return mock_serial;
}

TEST_CASE("RobotController class", "[RobotController]")
{
    StatusUpdater s;
    RobotController r = RobotController(s);

    r.moveToPosition(1,1,1);
    r.update();

    REQUIRE(r.isTrajectoryRunning() == true);    
}

TEST_CASE("Test motor enable and disable", "[RobotController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    StatusUpdater s;
    RobotController r = RobotController(s);

    mock_serial->purge_data();
    r.enableAllMotors();
    REQUIRE(mock_serial->mock_rcv() == "Power:ON");

    mock_serial->purge_data();
    r.disableAllMotors();
    REQUIRE(mock_serial->mock_rcv() == "Power:OFF");
}