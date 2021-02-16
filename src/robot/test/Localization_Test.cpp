#include <Catch/catch.hpp>

#include "Localization.h"
#include "test-utils.h"

void moveToPositionWithVelOnly(Localization& L, Point target)
{
    Velocity v{target.x, target.y, target.a};
    L.updateVelocityReading(v, 1);
    L.forceZeroVelocity();
}


TEST_CASE("SimpleVelocityOnly", "[Localization]")
{
    // Setup x,y offset for mm to be 0 for this simple case
    SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
    SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);
    
    Localization L;

    L.updateVelocityReading({1,0,0}, 1);
    REQUIRE(L.getVelocity() == Velocity{1,0,0});
    REQUIRE(L.getPosition() == Point{1,0,0});
}

TEST_CASE("SimplePositionAdjustment", "[Localization]")
{
    // Use fake perfect motion for simplicity
    SafeConfigModifier<bool> config_modifier("motion.fake_perfect_motion", true);

    SECTION("Zero Vel, Same Position, No offset")
    {
        // Setup x,y offset for mm to be 0 for this simple case
        SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
        SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);
        Localization L;

        // "Move" robot to target position
        float move_x = 1.0;
        float move_y = 0.0;
        float move_a = 0.0;
        Point move_target = {move_x, move_y, move_a};
        moveToPositionWithVelOnly(L, move_target);

        // Verify robot is in expected position
        Point position = L.getPosition();
        REQUIRE(position.x == Approx(move_x).margin(0.0005));
        REQUIRE(position.y == Approx(move_y).margin(0.0005));
        REQUIRE(position.a == Approx(move_a).margin(0.0005));

        // Send position update that should match current position
        L.updatePositionReading(move_target);
        
        // Verify position has not changed
        position = L.getPosition();
        REQUIRE(position.x == Approx(move_x).margin(0.0005));
        REQUIRE(position.y == Approx(move_y).margin(0.0005));
        REQUIRE(position.a == Approx(move_a).margin(0.0005));
    }

    SECTION("Zero Vel, Position adjustment, No offset")
    {
        // Setup x,y offset for mm to be 0 for this simple case
        SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
        SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);
        Localization L;

        // "Move" robot to target position
        float move_x = 1.0;
        float move_y = 0.0;
        float move_a = 0.0;
        Point move_target = {move_x, move_y, move_a};
        moveToPositionWithVelOnly(L, move_target);

        // Verify robot is in expected position
        Point position = L.getPosition();
        REQUIRE(position.x == Approx(move_x).margin(0.0005));
        REQUIRE(position.y == Approx(move_y).margin(0.0005));
        REQUIRE(position.a == Approx(move_a).margin(0.0005));

        // Send position update that should trigger adjustment
        float update_x = 2.0;
        float update_y = 0.0;
        float update_a = 0.0;
        Point update_target = {update_x, update_y, update_a};
        L.updatePositionReading(update_target);
        
        // Verify position changed as expected
        const float update_fraction_at_zero_vel = cfg.lookup("localization.update_fraction_at_zero_vel");
        float expected_x = update_fraction_at_zero_vel * (update_x - move_x) + move_x;
        position = L.getPosition();
        REQUIRE(position.x == Approx(expected_x).margin(0.0005));
        REQUIRE(position.y == Approx(move_y).margin(0.0005));
        REQUIRE(position.a == Approx(move_a).margin(0.0005));
    }

}

TEST_CASE("PositionUpdateWithMMOffset", "[Localization]")
{
    // Setup x,y offset for mm
    SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", -100.0);
    SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 10.0);
    Localization L;

    // "Move" robot to target position
    float move_x = 1.0;
    float move_y = 0.0;
    float move_a = 3.14;
    Point move_target = {move_x, move_y, move_a};
    moveToPositionWithVelOnly(L, move_target);

    // Verify robot is in expected position
    Point position = L.getPosition();
    REQUIRE(position.x == Approx(move_x).margin(0.0005));
    REQUIRE(position.y == Approx(move_y).margin(0.0005));
    REQUIRE(position.a == Approx(move_a).margin(0.0005));

    // Send position update that should not trigger adjustment
    // Since there is an offset, this position update needs to account for that
    float update_x = 1.1;
    float update_y = -0.01;
    float update_a = 3.14;
    Point update_target = {update_x, update_y, update_a};
    L.updatePositionReading(update_target);
    
    // Verify position did not change
    position = L.getPosition();
    REQUIRE(position.x == Approx(move_x).margin(0.0005));
    REQUIRE(position.y == Approx(move_y).margin(0.0005));
    REQUIRE(position.a == Approx(move_a).margin(0.0005));
}


TEST_CASE("PositionUpdateWithVelocityAdjustment", "[Localization]") 
{

    // Setup x,y offset for mm to be 0
    SafeConfigModifier<float> mm_x_config_modifier("localization.mm_x_offset", 0.0);
    SafeConfigModifier<float> mm_y_config_modifier("localization.mm_y_offset", 0.0);
    // Fix these values for this test so that I know exactly what they are. This test shouldn't break
    // because I modified those values
    SafeConfigModifier<float> frac_config_modifier("localization.update_fraction_at_zero_vel", 1.0);
    SafeConfigModifier<float> val_config_modifier("localization.val_for_zero_update", 0.1);
    Localization L;

    // Start 'robot' moving at specific velocity so we can inject the test conditions
    float target_vel_x = 0.05;
    float target_vel_y = 0.0;
    float target_vel_a = 0.0;
    L.updateVelocityReading({target_vel_x,target_vel_y,target_vel_a}, 1);

    // Send position update that should trigger adjustment, but will be adjusted due to velocity
    Point position = L.getPosition();
    float cur_x = position.x;
    float cur_y = position.y;
    float update_x = cur_x;  // This tests that an exact match of the position doesn't cause a change
    float update_y = 1.0;    // This tests that a difference causes an adjustment modified by velocity
    float update_a = 0.0;
    L.updatePositionReading({update_x, update_y, update_a});
    
    // Verify position changed as expected
    position = L.getPosition();
    const float val_for_zero_update = cfg.lookup("localization.val_for_zero_update");
    float expected_y = (target_vel_x / val_for_zero_update) * (update_y - cur_y) + cur_y;
    REQUIRE(position.x == Approx(cur_x).margin(0.0005));
    REQUIRE(position.y == Approx(expected_y).margin(0.002)); // Some extra margin here because the robot is still moving
    REQUIRE(position.a == Approx(update_a).margin(0.0005));
}