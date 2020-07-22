#include <Catch/catch.hpp>
#include <unistd.h>

#include "utils.h"

TEST_CASE("Sign util test", "[utils]") 
{
    REQUIRE(sgn(1) == 1);
    REQUIRE(sgn(-1) == -1);
    REQUIRE(sgn(0) == 0);
    REQUIRE(sgn(1.234) == 1);
    REQUIRE(sgn(-2.854) == -1);
    REQUIRE(sgn(-0.0000001) == -1);
}

TEST_CASE("Angle wrap test", "[utils]")
{
    REQUIRE(Approx(wrap_angle(M_PI-0.1)) == M_PI-0.1);
    REQUIRE(Approx(wrap_angle(-M_PI+0.1)) == -M_PI+0.1);
    REQUIRE(Approx(wrap_angle(0)) == 0);
    REQUIRE(Approx(wrap_angle(2*M_PI)).margin(0.0001) == 0);
    REQUIRE(Approx(wrap_angle(1.5*M_PI)) == -0.5*M_PI);
    REQUIRE(Approx(wrap_angle(-1.5*M_PI)) == 0.5*M_PI);
    REQUIRE(Approx(wrap_angle(4.5*M_PI)) == 0.5*M_PI);
}

TEST_CASE("Angle diff test", "[utils]")
{
    REQUIRE(angle_diff(0,0) == 0);
    REQUIRE(angle_diff(M_PI,M_PI) == 0);
    REQUIRE(Approx(angle_diff(0,2*M_PI)).margin(0.0001) == 0);
    REQUIRE(Approx(angle_diff(1.5*M_PI,0.5*M_PI)) == M_PI);
    REQUIRE(Approx(angle_diff(1.5*M_PI,3*M_PI)) == 0.5*M_PI);
    REQUIRE(Approx(angle_diff(-1.5*M_PI,3*M_PI)) == -0.5*M_PI);
}


// TODO: Fix problem with using unsigned long as time
// Note - either convert times in code to use chrono and better units, or 
// change millis to count from program execturion time
// TEST_CASE("Millis test", "[utils]")
// {
//     REQUIRE(millis() > 1595343137519); // current millis when this was written
//     REQUIRE(millis() < 1732953600000); // millis in 2025

//     unsigned long curMillis = millis();
//     usleep(10000); // 10 ms
//     REQUIRE(millis() - curMillis > 9);
//     REQUIRE(millis() - curMillis < 100);
// }