#include <Catch/catch.hpp>

#include "Distance.h"
#include "test-utils.h"


TEST_CASE("Distance start", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    Distance d;

    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");
}

TEST_CASE("Distance stop", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    Distance d;

    d.stop();
    REQUIRE(d.isRunning() == false);
    REQUIRE(mock_serial->mock_rcv_distance() == "stop");
}

TEST_CASE("Distance runninng nominal", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    Distance d;

    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send("dist:10,");
        d.checkForMeasurement();
    }

    REQUIRE(d.getDistance() == 10);
    
    d.stop();
    REQUIRE(d.isRunning() == false);
    REQUIRE(mock_serial->mock_rcv_distance() == "stop");
}

TEST_CASE("Distance runninng bad input", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    Distance d;

    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send("dist:-1,523,0.0");
        d.checkForMeasurement();
    }

    REQUIRE(d.getDistance() == 0);
    
    d.stop();
    REQUIRE(d.isRunning() == false);
    REQUIRE(mock_serial->mock_rcv_distance() == "stop");
}