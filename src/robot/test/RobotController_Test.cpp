#include <Catch/catch.hpp>

#include "RobotController.h"
#include "StatusUpdater.h"
#include "test-utils.h"


// Helper that commands the robot to move to the given position
// Returns the number of loops run before the robot either stopped or max_loops was hit
int moveToPositionHelper(RobotController& r, float x, float y, float a, int max_loops)
{
    MockSerialComms* mock_serial = build_and_get_mock_serial(CLEARCORE_USB);;
    MockClockWrapper* mock_clock = get_mock_clock();
    
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
    return count;
}

// A simple position test that checks if the target position is reached within
// a set amount of time
void coarsePositionTest(float x, float y, float a)
{
    // Force all control parameters to 0 for perfect velocity tracking (due to ffd)
    SafeConfigModifier<float> pos_kp_config_modifier("motion.translation.gains.kp", 0.0);
    SafeConfigModifier<float> pos_ki_config_modifier("motion.translation.gains.ki", 0.0);
    SafeConfigModifier<float> pos_kd_config_modifier("motion.translation.gains.kd", 0.0);
    SafeConfigModifier<float> ang_kp_config_modifier("motion.rotation.gains.kp", 0.0);
    SafeConfigModifier<float> ang_ki_config_modifier("motion.rotation.gains.ki", 0.0);
    SafeConfigModifier<float> ang_kd_config_modifier("motion.rotation.gains.kd", 0.0);
    SafeConfigModifier<float> limit_config_modifier("motion.limit_max_fraction", 0.8);
    
    int max_loop_counts =  100000;
    reset_mock_clock(); // Need to reset before creating RobotController which instantiates timers
    StatusUpdater s;
    RobotController r = RobotController(s);

    int num_loops = moveToPositionHelper(r, x, y, a, max_loop_counts);

    CHECK(num_loops != max_loop_counts);
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
    MockSerialComms* mock_serial = build_and_get_mock_serial(CLEARCORE_USB);;
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
    
    SECTION("Straight forward")
    {
        coarsePositionTest(0.5,0,0);
    }
    SECTION("Diagonal")
    {
        coarsePositionTest(0.5,0.5,0);
    }
    SECTION("Positive rotation")
    {
        coarsePositionTest(0,0,0.5);
    }
    SECTION("Negative rotation")
    {
        coarsePositionTest(0,0,-0.5);
    }
    SECTION("All directions")
    {
        coarsePositionTest(0.1,0.1,-0.5);
    }
}

void fakeMotionHelper(float x, float y, float a, int max_loops, StatusUpdater& s)
{
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
    MockSerialComms* mock_serial = build_and_get_mock_serial(CLEARCORE_USB);;
    mock_serial->purge_data();
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

    {
        // Enable fake_perfect_motion with config modifier, which should reset when going out of scope
        SafeConfigModifier<bool> config_modifier("motion.fake_perfect_motion", true);
        // Now this should work
        fakeMotionHelper(x,y,a,test_counts,s);
        status = s.getStatus();
        REQUIRE(status.pos_x == Approx(x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(a).margin(0.0005));
    }

    // Fake perfect motion should be reset, now it shouldn't move again
    fakeMotionHelper(x,y,a,test_counts,s);
    status = s.getStatus();
    REQUIRE(status.pos_x != Approx(x).margin(0.0005));
    REQUIRE(status.pos_y != Approx(y).margin(0.0005));
    REQUIRE(status.pos_a != Approx(a).margin(0.0005));
}




// Trajectory generation can no longer trigger an error. Re-enable this test once something can trigger errors again
// TEST_CASE("Block on error", "[RobotController]")
// {
//     MockSerialComms* mock_serial = build_and_get_mock_serial(CLEARCORE_USB);;
//     StatusUpdater s;
//     RobotController r = RobotController(s);

//     mock_serial->purge_data();
//     r.moveToPosition(0.00001,10000,0.0000001);
//     REQUIRE(s.getErrorStatus() == true);
//     REQUIRE(s.getInProgress() == false);
//     REQUIRE(r.isTrajectoryRunning() == false); 
//     r.update();
//     REQUIRE(mock_serial->mock_rcv_base() != "Power:ON");
//     REQUIRE(s.getErrorStatus() == true);
//     REQUIRE(s.getInProgress() == false);
//     REQUIRE(r.isTrajectoryRunning() == false); 
// }

// TEST_CASE("Benchmarking motion", "[RobotController]")
// {
//     MockSerialComms* mock_serial = build_and_get_mock_serial(CLEARCORE_USB);;
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