#include <Catch/catch.hpp>

#include "camera_tracker/CameraPipeline.h"
#include "test-utils.h"


TEST_CASE("Robot coords from image coords", "[Camera]")
{
    SafeConfigModifier<bool> config_modifier_1("vision_tracker.debug.use_debug_image", true);
    SafeConfigModifier<std::string> config_modifier_2("vision_tracker.side.debug_image", "/home/pi/images/test/TestImage2.jpg");
    SafeConfigModifier<std::string> config_modifier_3("vision_tracker.rear.debug_image", "/home/pi/images/test/TestImage2.jpg");
    
    float side_offset_x = 0.0;
    float side_offset_y = -1.5;
    SafeConfigModifier<float> config_modifier_4("vision_tracker.physical.side.x_offset", side_offset_x);
    SafeConfigModifier<float> config_modifier_5("vision_tracker.physical.side.y_offset", side_offset_y);
    SafeConfigModifier<float> config_modifier_6("vision_tracker.physical.side.z_offset", 0.55);

    CameraPipeline c(CAMERA_ID::SIDE, /*start_thread=*/ false);

    SECTION("Side cam, center point")
    {
        float u = 320/2;
        float v = 240/2;
        Eigen::Vector2f xy_world = c.cameraToRobot({u,v});
        CHECK(xy_world[0] == Approx(side_offset_x).margin(0.2));
        CHECK(xy_world[1] == Approx(side_offset_y).margin(0.2));
    }
    
    SECTION("Side cam, top corner")
    {
        float u = 0;
        float v = 0;
        Eigen::Vector2f xy_world = c.cameraToRobot({u,v});
        CHECK(xy_world[0] > side_offset_x);
        CHECK(xy_world[1] < side_offset_y);
    }
}