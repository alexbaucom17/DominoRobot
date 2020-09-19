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

TEST_CASE("TimeRunningAverage - Test constant time pauses", "[utils]") 
{
    int window = 10;
    int sleep_time_us = 2000;
    TimeRunningAverage T = TimeRunningAverage(window);

    for(int i = 0; i < window+1; i++)
    {
        T.mark_point();
        usleep(sleep_time_us);
    }

    REQUIRE(T.get_ms() == sleep_time_us/1000);
    REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));

    SECTION("Wrap buffer")
    {
        for(int i = 0; i < window+1; i++)
        {
            T.mark_point();
            usleep(sleep_time_us);
        }

        REQUIRE(T.get_ms() == sleep_time_us/1000);
        REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));
    }
}

TEST_CASE("TimeRunningAverage - Test larger buffer", "[utils]") 
{
    int window = 100;
    int sleep_time_us = 1000;
    TimeRunningAverage T = TimeRunningAverage(window);

    for(int i = 0; i < window+1; i++)
    {
        T.mark_point();
        usleep(sleep_time_us);
    }

    REQUIRE(T.get_ms() == sleep_time_us/1000);
    REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));

    SECTION("Wrap buffer")
    {
        for(int i = 0; i < window+1; i++)
        {
            T.mark_point();
            usleep(sleep_time_us);
        }

        REQUIRE(T.get_ms() == sleep_time_us/1000);
        REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));
    }
}

TEST_CASE("TimeRunningAverage - Test no data", "[utils]")
{
    TimeRunningAverage T = TimeRunningAverage(100);
    REQUIRE(T.get_ms() == 0);
    REQUIRE(T.get_sec() == 0);
}

TEST_CASE("TimeRunningAverage - Variable timing", "[utils]")
{
    int window = 10;
    int sleep_time_us = 2000;
    TimeRunningAverage T = TimeRunningAverage(window);

    SECTION("Partial buffer")
    {

        for(int i = 0; i < 6; i++)
        {
            T.mark_point();
            usleep(sleep_time_us);
            sleep_time_us += 2000;
        }
        REQUIRE(T.get_ms() == 5);

        SECTION("Full buffer")
        {

            for(int i = 0; i < 5; i++)
            {
                T.mark_point();
                usleep(sleep_time_us);
                sleep_time_us += 2000;
            }
            REQUIRE(T.get_ms() == 11);
        }
    }
}

TEST_CASE("Rate controller - slow", "[utils]")
{
    RateController r(1);
    REQUIRE(r.ready() == false);
    usleep(500000);
    REQUIRE(r.ready() == false);
    usleep(500000);
    REQUIRE(r.ready() == true);
    REQUIRE(r.ready() == false);
}

TEST_CASE("Rate controller - fast", "[utils]")
{
    RateController r(1000);
    REQUIRE(r.ready() == false);
    usleep(500);
    REQUIRE(r.ready() == false);
    usleep(500);
    REQUIRE(r.ready() == true);
    REQUIRE(r.ready() == false);
    usleep(1000);
    REQUIRE(r.ready() == true);
}
