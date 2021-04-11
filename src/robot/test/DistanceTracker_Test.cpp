#include <Catch/catch.hpp>

#include "distance_tracker/DistanceTracker.h"
#include "test-utils.h"

std::pair<float, float> pairedDistFromDistAngle(float target_dist, float target_angle, float o1, float o2)
{
    float d1 = target_dist + (o1-o2)/2 * tan(target_angle);
    float d2 = 2*target_dist - d1;
    return {d1, d2};
}


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

TEST_CASE("DistanceTracker nominal", "[distance]")
{
    constexpr float fwd_left_offset = 0.1;
    constexpr float fwd_right_offset = - 0.1;
    constexpr float side_front_offset = 0.1;
    constexpr float side_back_offset = - 0.1;
    constexpr float min_fwd_dist = 0.8;
    constexpr float max_fwd_dist = 1.2;
    constexpr float min_side_dist = 0.8;
    constexpr float max_side_dist = 1.2;
    
    // Ensure config is known
    SafeConfigModifier<int> config_modifier_1("distance_tracker.mapping.fwd_left", 0);
    SafeConfigModifier<int> config_modifier_2("distance_tracker.mapping.fwd_right", 1);
    SafeConfigModifier<int> config_modifier_3("distance_tracker.mapping.side_front", 2);
    SafeConfigModifier<int> config_modifier_4("distance_tracker.mapping.side_back", 3);
    SafeConfigModifier<float> config_modifier_6("distance_tracker.dimensions.fwd_left_offset", fwd_left_offset);
    SafeConfigModifier<float> config_modifier_7("distance_tracker.dimensions.fwd_right_offset", fwd_right_offset);
    SafeConfigModifier<float> config_modifier_8("distance_tracker.dimensions.side_front_offset", side_front_offset);
    SafeConfigModifier<float> config_modifier_9("distance_tracker.dimensions.side_back_offset", side_back_offset);
    SafeConfigModifier<int> config_modifier_10("distance_tracker.num_sensors", 4);
    SafeConfigModifier<int> config_modifier_11("distance_tracker.samples_to_average", 10);
    SafeConfigModifier<float> config_modifier_12("distance_tracker.limits.min_fwd_dist", min_fwd_dist);
    SafeConfigModifier<float> config_modifier_13("distance_tracker.limits.max_fwd_dist", max_fwd_dist);
    SafeConfigModifier<float> config_modifier_14("distance_tracker.limits.min_side_dist", min_side_dist);
    SafeConfigModifier<float> config_modifier_15("distance_tracker.limits.max_side_dist", max_side_dist);

    // Build unit under test
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    // Verify things started correctly
    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    // Set desired pose
    Point target_pose = {1, 1, 0};

    // Compute required distances from target pose
    auto [d1_fwd, d2_fwd] = pairedDistFromDistAngle(
        target_pose.x, target_pose.a, fwd_left_offset, fwd_right_offset);
    auto [d1_side, d2_side] = pairedDistFromDistAngle(
        target_pose.y, target_pose.a, side_front_offset, side_back_offset);
    // Put distances in string to simulate message from sensors
    char buff[100];
    snprintf(buff, sizeof(buff), "dist:%i,%i,%i,%i,",
        int(d1_fwd*1000), int(d2_fwd*1000), int(d1_side*1000) ,int(d2_side*1000.0));
    std::string data_to_send = buff;
    // Send the message a few times
    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send(data_to_send);
        d.checkForMeasurement();
    }

    Point pose = d.getDistancePose();
    CHECK(pose.x == Approx(target_pose.x).margin(0.001));
    CHECK(pose.y == Approx(target_pose.y).margin(0.001));
    CHECK(pose.a == Approx(target_pose.a).margin(0.02));
    
    d.stop();
    CHECK(d.isRunning() == false);
    CHECK(mock_serial->mock_rcv_distance() == "stop");
}

TEST_CASE("DistanceTracker angled", "[distance]")
{
    constexpr float fwd_left_offset = 0.1;
    constexpr float fwd_right_offset = - 0.1;
    constexpr float side_front_offset = 0.1;
    constexpr float side_back_offset = - 0.1;
    constexpr float min_fwd_dist = 0.8;
    constexpr float max_fwd_dist = 1.2;
    constexpr float min_side_dist = 0.8;
    constexpr float max_side_dist = 1.2;
    
    // Ensure config is known
    SafeConfigModifier<int> config_modifier_1("distance_tracker.mapping.fwd_left", 0);
    SafeConfigModifier<int> config_modifier_2("distance_tracker.mapping.fwd_right", 1);
    SafeConfigModifier<int> config_modifier_3("distance_tracker.mapping.side_front", 2);
    SafeConfigModifier<int> config_modifier_4("distance_tracker.mapping.side_back", 3);
    SafeConfigModifier<float> config_modifier_6("distance_tracker.dimensions.fwd_left_offset", fwd_left_offset);
    SafeConfigModifier<float> config_modifier_7("distance_tracker.dimensions.fwd_right_offset", fwd_right_offset);
    SafeConfigModifier<float> config_modifier_8("distance_tracker.dimensions.side_front_offset", side_front_offset);
    SafeConfigModifier<float> config_modifier_9("distance_tracker.dimensions.side_back_offset", side_back_offset);
    SafeConfigModifier<int> config_modifier_10("distance_tracker.num_sensors", 4);
    SafeConfigModifier<int> config_modifier_11("distance_tracker.samples_to_average", 10);
    SafeConfigModifier<float> config_modifier_12("distance_tracker.limits.min_fwd_dist", min_fwd_dist);
    SafeConfigModifier<float> config_modifier_13("distance_tracker.limits.max_fwd_dist", max_fwd_dist);
    SafeConfigModifier<float> config_modifier_14("distance_tracker.limits.min_side_dist", min_side_dist);
    SafeConfigModifier<float> config_modifier_15("distance_tracker.limits.max_side_dist", max_side_dist);

    // Build unit under test
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    // Verify things started correctly
    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    // Set desired pose
    Point target_pose = {1, 1, 5*M_PI/180.0};

    // Compute required distances from target pose
    auto [d1_fwd, d2_fwd] = pairedDistFromDistAngle(
        target_pose.x, target_pose.a, fwd_left_offset, fwd_right_offset);
    auto [d1_side, d2_side] = pairedDistFromDistAngle(
        target_pose.y, target_pose.a, side_front_offset, side_back_offset);
    // Put distances in string to simulate message from sensors
    char buff[100];
    snprintf(buff, sizeof(buff), "dist:%i,%i,%i,%i,",
        int(d1_fwd*1000), int(d2_fwd*1000), int(d1_side*1000) ,int(d2_side*1000.0));
    std::string data_to_send = buff;
    // Send the message a few times
    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send(data_to_send);
        d.checkForMeasurement();
    }

    Point pose = d.getDistancePose();
    CHECK(pose.x == Approx(target_pose.x).margin(0.001));
    CHECK(pose.y == Approx(target_pose.y).margin(0.001));
    CHECK(pose.a == Approx(target_pose.a).margin(0.02));
    
    d.stop();
    CHECK(d.isRunning() == false);
    CHECK(mock_serial->mock_rcv_distance() == "stop");
}

TEST_CASE("DistanceTracker angled2", "[distance]")
{
    constexpr float fwd_left_offset = 0.1;
    constexpr float fwd_right_offset = - 0.1;
    constexpr float side_front_offset = 0.1;
    constexpr float side_back_offset = - 0.1;
    constexpr float min_fwd_dist = 0.8;
    constexpr float max_fwd_dist = 1.5;
    constexpr float min_side_dist = 0.7;
    constexpr float max_side_dist = 1.2;
    
    // Ensure config is known
    SafeConfigModifier<int> config_modifier_1("distance_tracker.mapping.fwd_left", 0);
    SafeConfigModifier<int> config_modifier_2("distance_tracker.mapping.fwd_right", 1);
    SafeConfigModifier<int> config_modifier_3("distance_tracker.mapping.side_front", 2);
    SafeConfigModifier<int> config_modifier_4("distance_tracker.mapping.side_back", 3);
    SafeConfigModifier<float> config_modifier_6("distance_tracker.dimensions.fwd_left_offset", fwd_left_offset);
    SafeConfigModifier<float> config_modifier_7("distance_tracker.dimensions.fwd_right_offset", fwd_right_offset);
    SafeConfigModifier<float> config_modifier_8("distance_tracker.dimensions.side_front_offset", side_front_offset);
    SafeConfigModifier<float> config_modifier_9("distance_tracker.dimensions.side_back_offset", side_back_offset);
    SafeConfigModifier<int> config_modifier_10("distance_tracker.num_sensors", 4);
    SafeConfigModifier<int> config_modifier_11("distance_tracker.samples_to_average", 10);
    SafeConfigModifier<float> config_modifier_12("distance_tracker.limits.min_fwd_dist", min_fwd_dist);
    SafeConfigModifier<float> config_modifier_13("distance_tracker.limits.max_fwd_dist", max_fwd_dist);
    SafeConfigModifier<float> config_modifier_14("distance_tracker.limits.min_side_dist", min_side_dist);
    SafeConfigModifier<float> config_modifier_15("distance_tracker.limits.max_side_dist", max_side_dist);

    // Build unit under test
    MockSerialComms* mock_serial = build_and_get_mock_serial(ARDUINO_USB);;
    DistanceTracker d;

    // Verify things started correctly
    d.start();
    REQUIRE(d.isRunning() == true);
    REQUIRE(mock_serial->mock_rcv_distance() == "start");

    // Set desired pose
    Point target_pose = {1.3, 0.8, -10*M_PI/180.0};

    // Compute required distances from target pose
    auto [d1_fwd, d2_fwd] = pairedDistFromDistAngle(
        target_pose.x, target_pose.a, fwd_left_offset, fwd_right_offset);
    auto [d1_side, d2_side] = pairedDistFromDistAngle(
        target_pose.y, target_pose.a, side_front_offset, side_back_offset);
    // Put distances in string to simulate message from sensors
    char buff[100];
    snprintf(buff, sizeof(buff), "dist:%i,%i,%i,%i,",
        int(d1_fwd*1000), int(d2_fwd*1000), int(d1_side*1000) ,int(d2_side*1000.0));
    std::string data_to_send = buff;
    // Send the message a few times
    for(int i = 0; i < 10; i++)
    {
        mock_serial->mock_send(data_to_send);
        d.checkForMeasurement();
    }

    Point pose = d.getDistancePose();
    CHECK(pose.x == Approx(target_pose.x).margin(0.001));
    CHECK(pose.y == Approx(target_pose.y).margin(0.001));
    CHECK(pose.a == Approx(target_pose.a).margin(0.02));
    
    d.stop();
    CHECK(d.isRunning() == false);
    CHECK(mock_serial->mock_rcv_distance() == "stop");
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