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
        CHECK(xy_world[0] < side_offset_x);
        CHECK(xy_world[1] > side_offset_y);
    }
}

// Verify that camera params between actual and test config are synced if this test is giving trouble
TEST_CASE("Marker Detection", "[Camera]")
{
    struct TestImageMetadata
    {
        std::string name;
        bool expected_detection;
        CAMERA_ID id;
        Eigen::Vector2f expected_point_px = {-1, -1};
    };

    std::vector<TestImageMetadata> test_images = {
        {.name = "20210712175430_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712175436_rear_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::REAR},
        {.name = "20210712175519_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712175525_rear_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::REAR},
        {.name = "20210712175606_side_img_raw.jpg",
        .expected_detection = false,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712175612_rear_img_raw.jpg",
        .expected_detection = false,
        .id = CAMERA_ID::REAR},
        {.name = "20210712175719_side_img_raw.jpg",
        .expected_detection = false,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712175728_rear_img_raw.jpg",
        .expected_detection = false,
        .id = CAMERA_ID::REAR},
        {.name = "20210712175942_rear_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::REAR},
        {.name = "20210712180047_rear_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::REAR},
        {.name = "20210712180208_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712180322_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712180446_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712180528_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
        {.name = "20210712182429_side_img_raw.jpg",
        .expected_detection = true,
        .id = CAMERA_ID::SIDE},
    };

    for (const auto& item : test_images) 
    {
        std::string image_dir = "/home/pi/DominoRobot/src/robot/test/testdata/new_images/";
        std::string image_path = image_dir + item.name;
        SafeConfigModifier<bool> config_modifier_1("vision_tracker.debug.use_debug_image", true);
        SafeConfigModifier<std::string> config_modifier_2("vision_tracker.side.debug_image", image_path);
        SafeConfigModifier<std::string> config_modifier_3("vision_tracker.rear.debug_image", image_path);
       
        CameraPipeline c(item.id, /*start_thread=*/ false);
        c.oneLoop();
        CameraPipelineOutput output = c.getData();

        PLOGI << "Checking detection in image " << item.name;
        CHECK(output.ok == item.expected_detection);
        if(item.expected_point_px != Eigen::Vector2f({-1,-1}))
        {
            CHECK(output.uv == item.expected_point_px);
        }
    }
}