#include <Catch/catch.hpp>

#include "RobotController.h"
#include "StatusUpdater.h"
#include "serial/MockSerialComms.h"
#include "serial/SerialCommsFactory.h"
#include "constants.h"
#include <unistd.h>

MockSerialComms* build_and_get_mock_serial()
{
    SerialCommsBase* base_serial = SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB);
    // Slightly dangerous....
    MockSerialComms* mock_serial = dynamic_cast<MockSerialComms*>(base_serial);
    return mock_serial;
}

void coarsePositionTest(float x, float y, float a, int max_loops)
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    StatusUpdater s;
    RobotController r = RobotController(s);

    mock_serial->purge_data();
    r.moveToPosition(x,y,a);
    REQUIRE(mock_serial->mock_rcv_base() == "Power:ON");

    int count = 0;
    while(r.isTrajectoryRunning() && count < max_loops)
    {
        count++;

        r.update();
        // Report back the exact velocity commanded
        std::string cmd_vel = mock_serial->mock_rcv();
        mock_serial->mock_send("base" + cmd_vel);

        usleep(100);
    }

    CHECK(count != max_loops);
    CHECK(r.isTrajectoryRunning() == false);

    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.pos_x == Approx(x).margin(0.0005));
    REQUIRE(status.pos_y == Approx(y).margin(0.0005));
    REQUIRE(status.pos_a == Approx(a).margin(0.0005));
}

TEST_CASE("RobotController class", "[RobotController]")
{
    StatusUpdater s;
    RobotController r = RobotController(s);

    r.moveToPosition(1,1,1);
    r.update();

    REQUIRE(r.isTrajectoryRunning() == true);    
}

TEST_CASE("Motor enable and disable", "[RobotController]")
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

TEST_CASE("Simple coarse motion", "[RobotController]")
{
    int test_counts =  10000;
    SECTION("Straight forward")
    {
        coarsePositionTest(0.5,0,0, test_counts);
    }
    SECTION("Diagonal")
    {
        coarsePositionTest(0.5,0.5,0, test_counts);
    }
    SECTION("Positive rotation")
    {
        coarsePositionTest(0,0,0.5, test_counts);
    }
    SECTION("Negative rotation")
    {
        coarsePositionTest(0,0,-0.5, test_counts);
    }
    SECTION("All directions")
    {
        coarsePositionTest(0.1,0.1,-0.5, test_counts);
    }
}

TEST_CASE("Block on error", "[RobotController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    StatusUpdater s;
    RobotController r = RobotController(s);

    mock_serial->purge_data();
    r.moveToPosition(0.00001,10000,0.0000001);
    REQUIRE(s.getErrorStatus() == true);
    REQUIRE(s.getInProgress() == false);
    REQUIRE(r.isTrajectoryRunning() == false); 
    r.update();
    REQUIRE(mock_serial->mock_rcv() != "Power:ON");
    REQUIRE(s.getErrorStatus() == true);
    REQUIRE(s.getInProgress() == false);
    REQUIRE(r.isTrajectoryRunning() == false); 
}

// TEST_CASE("Benchmarking motion", "[RobotController]")
// {
//     MockSerialComms* mock_serial = build_and_get_mock_serial();
//     StatusUpdater s;
//     RobotController r = RobotController(s);

//     mock_serial->purge_data();
//     r.moveToPosition(10,0,0);
//     REQUIRE(mock_serial->mock_rcv() == "Power:ON");

//     BENCHMARK("RobotController benchmark")
//     {
//         r.update();
//         // Report back the exact velocity commanded
//         std::string cmd_vel = mock_serial->mock_rcv();
//         mock_serial->mock_send(cmd_vel);
//     };
// }