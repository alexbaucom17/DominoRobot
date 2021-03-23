#include <Catch/catch.hpp>

#include "distance_tracker/DistanceTracker.h"
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
    // Ensure config is known
    SafeConfigModifier<int> config_modifier_1("distance_tracker.mapping.fwd_left", 0);
    SafeConfigModifier<int> config_modifier_2("distance_tracker.mapping.fwd_right", 1);
    SafeConfigModifier<int> config_modifier_3("distance_tracker.mapping.angled_left", 2);
    SafeConfigModifier<int> config_modifier_4("distance_tracker.mapping.angled_right", 3);
    SafeConfigModifier<float> config_modifier_5("distance_tracker.dimensions.angle_from_fwd_degrees", -45);
    SafeConfigModifier<float> config_modifier_6("distance_tracker.dimensions.left_fwd_offset", 0.1);
    SafeConfigModifier<float> config_modifier_7("distance_tracker.dimensions.right_fwd_offset", -0.1);
    SafeConfigModifier<float> config_modifier_8("distance_tracker.dimensions.left_angle_offset", 0.1);
    SafeConfigModifier<float> config_modifier_9("distance_tracker.dimensions.right_angle_offset", -0.1);
    SafeConfigModifier<int> config_modifier_10("distance_tracker.num_sensors", 4);
    SafeConfigModifier<int> config_modifier_11("distance_tracker.samples_to_average", 10);

    // Build unit under test
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    // Verify things started correctly
    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    // Send simple data that should show the robot at exactly [1m, -1m, 0 deg]
    std::string data_to_send = "dist:1000,1000,1555,1273,";
    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send(data_to_send);
        d.checkForMeasurement();
    }

    Point pose = d.getDistancePose();
    REQUIRE(pose.x == Approx(1.0).margin(0.001));
    REQUIRE(pose.y == Approx(-1.0).margin(0.001));
    REQUIRE(pose.a == Approx(0.0).margin(0.01));
    
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

    Point pose = d.getDistancePose();
    REQUIRE(pose.x == Approx(0.0).margin(0.0005));
    REQUIRE(pose.y == Approx(0.0).margin(0.0005));
    REQUIRE(pose.a == Approx(0.0).margin(0.0005));
    
    d.stop();
    REQUIRE(d.isRunning() == false);
    REQUIRE(mock_serial->mock_rcv_distance() == "stop");
}