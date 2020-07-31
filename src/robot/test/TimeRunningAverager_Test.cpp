#include <Catch/catch.hpp>
#include <unistd.h>

#include "TimeRunningAverage.h"


TEST_CASE("Test constant time pauses", "[averager]") 
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

TEST_CASE("Test larger buffer", "[averager]") 
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

TEST_CASE("Test no data", "[averager]")
{
    TimeRunningAverage T = TimeRunningAverage(100);
    REQUIRE(T.get_ms() == 0);
    REQUIRE(T.get_sec() == 0);
}

TEST_CASE("Variable timing", "[averager]")
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