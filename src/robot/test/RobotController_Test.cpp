#include <Catch/catch.hpp>

#include "RobotController.h"
#include "StatusUpdater.h"
#include "test-utils.h"

void coarsePositionTest(float x, float y, float a, int max_loops)
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    MockClockWrapper* mock_clock = get_mock_clock();
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
        std::string cmd_vel = mock_serial->mock_rcv_base();
        mock_serial->mock_send("base:" + cmd_vel);

        mock_clock->advance_ms(1);
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
    REQUIRE(mock_serial->mock_rcv_base() == "Power:ON");

    mock_serial->purge_data();
    r.disableAllMotors();
    REQUIRE(mock_serial->mock_rcv_base() == "Power:OFF");
}

TEST_CASE("Simple coarse motion", "[RobotController]")
{
    int test_counts =  100000;
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
    REQUIRE(mock_serial->mock_rcv_base() != "Power:ON");
    REQUIRE(s.getErrorStatus() == true);
    REQUIRE(s.getInProgress() == false);
    REQUIRE(r.isTrajectoryRunning() == false); 
}


void fakeMotionHelper(float x, float y, float a, int max_loops, StatusUpdater& s)
{
    MockClockWrapper* mock_clock = get_mock_clock();
    RobotController r = RobotController(s);
    r.moveToPosition(x,y,a);

    int count = 0;
    while(r.isTrajectoryRunning() && count < max_loops)
    {
        count++;
        r.update();
        mock_clock->advance_us(1000);
    }
}

TEST_CASE("Validate fake_perfect_motion option", "[RobotController]")
{
    StatusUpdater s;
    float x = 0.5;
    float y = 0.5;
    float a = 0.5;
    int test_counts = 10000;

    // Prior to fake motion enabled, the robot should not move
    fakeMotionHelper(x,y,a,test_counts,s);
    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.pos_x != Approx(x).margin(0.0005));
    REQUIRE(status.pos_y != Approx(y).margin(0.0005));
    REQUIRE(status.pos_a != Approx(a).margin(0.0005));

    // Enable fake_perfect_motion
    libconfig::Setting& fake_perfect_motion = cfg.lookup("motion.fake_perfect_motion");
    fake_perfect_motion = true;
    // Now this should work
    fakeMotionHelper(x,y,a,test_counts,s);
    status = s.getStatus();
    REQUIRE(status.pos_x == Approx(x).margin(0.0005));
    REQUIRE(status.pos_y == Approx(y).margin(0.0005));
    REQUIRE(status.pos_a == Approx(a).margin(0.0005));

    // Disable fake perfect motion, now it shouldn't move again
    fake_perfect_motion = false;
    fakeMotionHelper(x,y,a,test_counts,s);
    status = s.getStatus();
    REQUIRE(status.pos_x != Approx(x).margin(0.0005));
    REQUIRE(status.pos_y != Approx(y).margin(0.0005));
    REQUIRE(status.pos_a != Approx(a).margin(0.0005));
}

// TEST_CASE("Benchmarking motion", "[RobotController]")
// {
//     MockSerialComms* mock_serial = build_and_get_mock_serial();
//     StatusUpdater s;
//     RobotController r = RobotController(s);

//     mock_serial->purge_data();
//     r.moveToPosition(10,0,0);
//     REQUIRE(mock_serial->mock_rcv_base() == "Power:ON");

//     BENCHMARK("RobotController benchmark")
//     {
//         r.update();
//         // Report back the exact velocity commanded
//         std::string cmd_vel = mock_serial->mock_rcv_base();
//         mock_serial->mock_send(cmd_vel);
//     };
// }