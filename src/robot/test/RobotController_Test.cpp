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

TEST_CASE("Distance controller simple", "[RobotController]")
{
    // Configure parameters and mocks
    SafeConfigModifier<bool> config_modifier("motion.fake_perfect_motion", true);
    DistanceTrackerMock* distance_tracker_mock = build_and_get_mock_distance_tracker();
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();

    // Setup unit for test
    StatusUpdater s;
    RobotController r = RobotController(s);
    float dist_x = 0.6;
    float dist_y = 0.0;
    float dist_a = 0.0;
    float initial_distance = 1.0;
    r.moveWithDistance(dist_x, dist_y, dist_a);
    REQUIRE(distance_tracker_mock->isRunning() == true);
    distance_tracker_mock->setMockDistancePose({initial_distance,0,0});

    // Movement should not have started yet as it is waiting for distance values
    mock_clock->advance_ms(1);
    r.update();
    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.pos_x == Approx(0.0).margin(0.0005));
    REQUIRE(status.pos_y == Approx(0.0).margin(0.0005));
    REQUIRE(status.pos_a == Approx(0.0).margin(0.0005));

    // Run updates until trajectory stops
    mock_clock->advance_sec(1);
    for (int i = 0; i < 1000; i++) 
    {
        float new_dist = std::max(dist_x + (1-i/200.0f)*(initial_distance-dist_x), dist_x);
        distance_tracker_mock->setMockDistancePose({new_dist,0,0});
        r.update();
        mock_clock->advance_ms(5);

        if(!r.isTrajectoryRunning()) break;
    }

    r.update();

    // Make sure trajectory reaches target
    status = s.getStatus();
    CHECK(status.pos_x == Approx(initial_distance - dist_x).margin(0.0005));
    CHECK(status.pos_y == Approx(dist_y).margin(0.0005));
    CHECK(status.pos_a == Approx(dist_a).margin(0.0005));
    CHECK(distance_tracker_mock->isRunning() == false);
    CHECK(r.isTrajectoryRunning() == false);
}



int positionUpdateTestingHelper(RobotController& r, float x, float y, float a, int max_loops)
{
    MockClockWrapper* mock_clock = get_mock_clock();
    r.moveToPosition(x,y,a);
    int count = 0;
    while(r.isTrajectoryRunning() && count < max_loops)
    {
        count++;
        r.update();
        mock_clock->advance_ms(1);
    }
    return count;
}

TEST_CASE("Position update", "[RobotController]") 
{

    // Use fake perfect motion for simplicity
    SafeConfigModifier<bool> config_modifier("motion.fake_perfect_motion", true);

    SECTION("Zero Vel, Same Position, No offset")
    {
        // Setup x,y offset for mm to be 0 for this simple case
        SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
        SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);

        // Setup objects
        int max_loop_counts =  100000;
        reset_mock_clock(); // Need to reset before creating RobotController which instantiates timers
        StatusUpdater s;
        RobotController r = RobotController(s);

        // Move robot to position and end with 0 vel
        float move_x = 1.0;
        float move_y = 0.0;
        float move_a = 0.0;
        int num_loops = positionUpdateTestingHelper(r, move_x, move_y, move_a, max_loop_counts);
        CHECK(num_loops != max_loop_counts);

        // Verify robot is in expected position
        StatusUpdater::Status status = s.getStatus();
        REQUIRE(status.pos_x == Approx(move_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));

        // Send position update that should match current position
        r.inputPosition(move_x, move_y, move_a);
        // Need to run update again for status updater to get the changes
        r.update();
        
        // Verify position has not changed
        status = s.getStatus();
        REQUIRE(status.pos_x == Approx(move_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));
    }

    SECTION("Zero Vel, Position adjustment, No offset")
    {
        // Setup x,y offset for mm to be 0 for this simple case
        SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
        SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);

        // Setup objects
        int max_loop_counts =  100000;
        reset_mock_clock(); // Need to reset before creating RobotController which instantiates timers
        StatusUpdater s;
        RobotController r = RobotController(s);

        // Move robot to position and end with 0 vel
        float move_x = 1.0;
        float move_y = 0.0;
        float move_a = 0.0;
        int num_loops = positionUpdateTestingHelper(r, move_x, move_y, move_a, max_loop_counts);
        CHECK(num_loops != max_loop_counts);

        // Verify robot is in expected position
        StatusUpdater::Status status = s.getStatus();
        REQUIRE(status.pos_x == Approx(move_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));

        // Send position update that should trigger adjustment
        float update_x = 2.0;
        float update_y = 0.0;
        float update_a = 0.0;
        r.inputPosition(update_x, update_y, update_a);
        // Need to run update again for status updater to get the changes
        r.update();
        status = s.getStatus();
        
        // Verify position changed as expected
        const float update_fraction_at_zero_vel = cfg.lookup("localization.update_fraction_at_zero_vel");
        float expected_x = update_fraction_at_zero_vel * (update_x - move_x) + move_x;
        REQUIRE(status.pos_x == Approx(expected_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));
    }

    SECTION("Zero Vel, No adjustment, With offset")
    {
        // Setup x,y offset for mm
        SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", -100.0);
        SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 10.0);

        // Setup objects
        int max_loop_counts =  100000;
        reset_mock_clock(); // Need to reset before creating RobotController which instantiates timers
        StatusUpdater s;
        RobotController r = RobotController(s);

        // Move robot to position and end with 0 vel
        float move_x = 1.0;
        float move_y = 0.0;
        float move_a = 3.14;
        int num_loops = positionUpdateTestingHelper(r, move_x, move_y, move_a, max_loop_counts);
        CHECK(num_loops != max_loop_counts);

        // Verify robot is in expected position
        StatusUpdater::Status status = s.getStatus();
        REQUIRE(status.pos_x == Approx(move_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));

        // Send position update that should trigger no adjustment
        // Since there is an offset, this position update needs to account for that
        float update_x = 1.1;
        float update_y = -0.01;
        float update_a = 3.14;
        r.inputPosition(update_x, update_y, update_a);
        // Need to run update again for status updater to get the changes
        r.update();
        status = s.getStatus();
        
        // Verify position did not change
        REQUIRE(status.pos_x == Approx(move_x).margin(0.0005));
        REQUIRE(status.pos_y == Approx(move_y).margin(0.0005));
        REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));
    }

    // SECTION("Non-zero Vel, Position adjustment, No offset")
    // {
    //     // Setup x,y offset for mm to be 0
    //     SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
    //     SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);
    //     // Fix these values for this test so that I know exactly what they are. This test shouldn't break
    //     // because I modified those values
    //     SafeConfigModifier<float> frac_config_modifier("localization.update_fraction_at_zero_vel", 1.0);
    //     SafeConfigModifier<float> val_config_modifier("localization.val_for_zero_update", 0.1);

    //     // Setup objects
    //     int max_loop_counts =  100000;
    //     reset_mock_clock(); // Need to reset before creating RobotController which instantiates timers
    //     StatusUpdater s;
    //     RobotController r = RobotController(s);

    //     float move_x = 1.0;
    //     float move_y = 0.0;
    //     float move_a = 0.0;
    //     float target_vel_x = 0.05;
    //     float target_vel_y = 0.0;
    //     float target_vel_a = 0.0;

    //     // Start robot moving and break when we reach a specific velocity so we can inject the test conditions
    //     MockClockWrapper* mock_clock = get_mock_clock();
    //     r.moveToPositionFine(move_x,move_y,move_a);
    //     int count = 0;
    //     while(r.isTrajectoryRunning() && count < max_loop_counts)
    //     {
    //         count++;
    //         r.update();
    //         mock_clock->advance_ms(1);

    //         StatusUpdater::Status status = s.getStatus();
    //         if (status.vel_x == Approx(target_vel_x).margin(0.0005) && 
    //             status.vel_y == Approx(target_vel_y).margin(0.0005) &&
    //             status.vel_a == Approx(target_vel_a).margin(0.0005) )
    //         {
    //             break;
    //         }
    //     }

    //     // Send position update that should trigger adjustment, but will be adjusted due to velocity
    //     StatusUpdater::Status status = s.getStatus();
    //     float cur_x = status.pos_x;
    //     float cur_y = status.pos_y;
    //     float update_x = cur_x;  // This tests that an exact match of the position doesn't cause a change
    //     float update_y = 1.0;    // This tests that a difference causes an adjustment modified by velocity
    //     float update_a = 0.0;
    //     r.inputPosition(update_x, update_y, update_a);
    //     // Need to run update again for status updater to get the changes
    //     r.update();
    //     status = s.getStatus();
        
    //     // Verify position changed as expected
    //     const float val_for_zero_update = cfg.lookup("localization.val_for_zero_update");
    //     float expected_y = (target_vel_x / val_for_zero_update) * (update_y - cur_y) + cur_y;
    //     REQUIRE(status.pos_x == Approx(cur_x).margin(0.0005));
    //     REQUIRE(status.pos_y == Approx(expected_y).margin(0.002)); // Some extra margin here because the robot is still moving
    //     REQUIRE(status.pos_a == Approx(move_a).margin(0.0005));
    // }
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