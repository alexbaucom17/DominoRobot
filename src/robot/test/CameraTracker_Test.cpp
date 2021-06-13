#include <Catch/catch.hpp>

#include "camera_tracker/CameraTracker.h"
#include "test-utils.h"
#include <unistd.h>

TEST_CASE("Nominal case", "[Camera]")
{
    SafeConfigModifier<bool> config_modifier_1("vision_tracker.debug.use_debug_image", true);
    SafeConfigModifier<std::string> config_modifier_2("vision_tracker.side.debug_image", "/home/pi/images/test/TestImage2.jpg");
    SafeConfigModifier<std::string> config_modifier_3("vision_tracker.rear.debug_image", "/home/pi/images/test/TestImage2.jpg");

    CameraTracker c(/*start_thread=*/ true);
    int counter = 0;
    CameraTrackerOutput output;
    output.ok = false;
    while(!output.ok && counter < 50){
        c.update();
        output = c.getPoseFromCamera();
        counter++;
        usleep(100000);
    }
    REQUIRE(output.ok == true);
    Point p = output.pose;
    bool any_nonzero = p.x != 0 || p.y != 0 || p.a != 0;
    CHECK(any_nonzero == true);
}

TEST_CASE("Robot pose from coords", "[Camera]")
{
    SafeConfigModifier<bool> config_modifier_1("vision_tracker.debug.use_debug_image", true);
    SafeConfigModifier<std::string> config_modifier_2("vision_tracker.side.debug_image", "/home/pi/images/test/TestImage2.jpg");
    SafeConfigModifier<std::string> config_modifier_3("vision_tracker.rear.debug_image", "/home/pi/images/test/TestImage2.jpg");
    
    float side_target_x = 0.0;
    float side_target_y = -1.4;
    SafeConfigModifier<float> config_modifier_4("vision_tracker.physical.side.x_offset", side_target_x);
    SafeConfigModifier<float> config_modifier_5("vision_tracker.physical.side.y_offset", side_target_y);
    SafeConfigModifier<float> config_modifier_6("vision_tracker.physical.side.z_offset", 0.55);
    SafeConfigModifier<float> config_modifier_7("vision_tracker.physical.side.target_x", side_target_x);
    SafeConfigModifier<float> config_modifier_8("vision_tracker.physical.side.target_y", side_target_y);
    float rear_target_x = -1.5;
    float rear_target_y = 0.0;
    SafeConfigModifier<float> config_modifier_9("vision_tracker.physical.rear.x_offset", rear_target_x);
    SafeConfigModifier<float> config_modifier_10("vision_tracker.physical.rear.y_offset", rear_target_y);
    SafeConfigModifier<float> config_modifier_11("vision_tracker.physical.rear.z_offset", 0.55);
    SafeConfigModifier<float> config_modifier_12("vision_tracker.physical.rear.target_x", rear_target_x);
    SafeConfigModifier<float> config_modifier_13("vision_tracker.physical.rear.target_y", rear_target_y);

    SECTION("Exact match, target at center images")
    {
        CameraTracker c(/*start_thread=*/ false);
        Point p = c.computeRobotPoseFromImagePoints({side_target_x, side_target_y},{rear_target_x,rear_target_y});
        CHECK(p.x == Approx(0).margin(0.0005));
        CHECK(p.y == Approx(0).margin(0.0005));
        CHECK(p.a == Approx(0).margin(0.0005));
    }
    SECTION("Offset translation, target at center images")
    {
        CameraTracker c(/*start_thread=*/ false);
        float x_offset = 0.1;
        float y_offset = -0.2;
        Point p = c.computeRobotPoseFromImagePoints({side_target_x + x_offset, side_target_y + y_offset},
                                                    {rear_target_x + x_offset, rear_target_y + y_offset});
        CHECK(p.x == Approx(-x_offset).margin(0.0005));
        CHECK(p.y == Approx(-y_offset).margin(0.0005));
        CHECK(p.a == Approx(0).margin(0.0005));
    }
    SECTION("Offset angle, target at center images")
    {
        CameraTracker c(/*start_thread=*/ false);
        float a_offset = 5 * M_PI / 180.0;
        Point p = c.computeRobotPoseFromImagePoints({side_target_y * -sin(a_offset), side_target_y * cos(a_offset) },
                                                    {rear_target_x * cos(a_offset), rear_target_x * sin(a_offset)});
        CHECK(p.x == Approx(0).margin(0.0005));
        CHECK(p.y == Approx(0).margin(0.0005));
        CHECK(p.a == Approx(-a_offset).margin(0.0005));
    }
}
