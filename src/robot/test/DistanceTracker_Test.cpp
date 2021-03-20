#include <Catch/catch.hpp>

#include "DistanceTracker.h"
#include "test-utils.h"


TEST_CASE("DistanceTracker start", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");
}

TEST_CASE("DistanceTracker stop", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    d.stop();
    REQUIRE(d.isRunning() == false);
    REQUIRE(mock_serial->mock_rcv_distance() == "stop");
}

TEST_CASE("DistanceTracker runninng nominal", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

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

TEST_CASE("DistanceTracker runninng bad input", "[distance]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

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