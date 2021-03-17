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