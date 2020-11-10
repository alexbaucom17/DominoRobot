#include <Catch/catch.hpp>
#include <unistd.h>

#include "utils.h"
#include "test-utils.h"

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
    MockClockWrapper* mock_clock = get_mock_clock();

    for(int i = 0; i < window+1; i++)
    {
        T.mark_point();
        mock_clock->advance_us(sleep_time_us);
    }

    REQUIRE(T.get_ms() == sleep_time_us/1000);
    REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));

    SECTION("Wrap buffer")
    {
        for(int i = 0; i < window+1; i++)
        {
            T.mark_point();
            mock_clock->advance_us(sleep_time_us);
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
    MockClockWrapper* mock_clock = get_mock_clock();

    for(int i = 0; i < window+1; i++)
    {
        T.mark_point();
        mock_clock->advance_us(sleep_time_us);
    }

    REQUIRE(T.get_ms() == sleep_time_us/1000);
    REQUIRE(T.get_sec() == Approx(sleep_time_us/1000000.0));

    SECTION("Wrap buffer")
    {
        for(int i = 0; i < window+1; i++)
        {
            T.mark_point();
            mock_clock->advance_us(sleep_time_us);
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
    MockClockWrapper* mock_clock = get_mock_clock();

    SECTION("Partial buffer")
    {

        for(int i = 0; i < 6; i++)
        {
            T.mark_point();
            mock_clock->advance_us(sleep_time_us);
            sleep_time_us += 2000;
        }
        REQUIRE(T.get_ms() == 5);

        SECTION("Full buffer")
        {

            for(int i = 0; i < 5; i++)
            {
                T.mark_point();
                mock_clock->advance_us(sleep_time_us);
                sleep_time_us += 2000;
            }
            REQUIRE(T.get_ms() == 11);
        }
    }
}

TEST_CASE("Rate controller - slow", "[utils]")
{
    MockClockWrapper* mock_clock = get_mock_clock();
    RateController r(1);
    REQUIRE(r.ready() == false);
    mock_clock->advance_sec(0.5);
    REQUIRE(r.ready() == false);
    mock_clock->advance_sec(0.55);
    REQUIRE(r.ready() == true);
    REQUIRE(r.ready() == false);
}

TEST_CASE("Rate controller - fast", "[utils]")
{
    MockClockWrapper* mock_clock = get_mock_clock();
    RateController r(1000);
    REQUIRE(r.ready() == false);
    mock_clock->advance_us(500);
    REQUIRE(r.ready() == false);
    mock_clock->advance_us(600);
    REQUIRE(r.ready() == true);
    REQUIRE(r.ready() == false);
    mock_clock->advance_us(1100);
    REQUIRE(r.ready() == true);
}


TEST_CASE("MockClockWrapper", "[utils]")
{
    MockClockWrapper* mock_clock = get_mock_clock();
    SECTION("Microseconds")
    {
        int count = 100;
        ClockTimePoint point = mock_clock->now();
        mock_clock->advance_us(count);
        ClockTimePoint new_point = mock_clock->now();
        std::chrono::microseconds test_duration = 
            std::chrono::duration_cast<std::chrono::microseconds>(new_point - point);
        REQUIRE(test_duration.count() == count);
    }
    SECTION("Milliseconds")
    {
        int count = 100;
        ClockTimePoint point = mock_clock->now();
        mock_clock->advance_ms(count);
        ClockTimePoint new_point = mock_clock->now();
        std::chrono::milliseconds test_duration = 
            std::chrono::duration_cast<std::chrono::milliseconds>(new_point - point);
        REQUIRE(test_duration.count() == count);
    }
    SECTION("Seconds")
    {
        float count = 5.2;
        ClockTimePoint point = mock_clock->now();
        mock_clock->advance_sec(count);
        ClockTimePoint new_point = mock_clock->now();
        FpSeconds test_duration = 
            std::chrono::duration_cast<FpSeconds>(new_point - point);
        REQUIRE(test_duration.count() == count);
    }
    SECTION("Set")
    {
        ClockTimePoint point = std::chrono::steady_clock::now();
        mock_clock->set(point);
        REQUIRE(mock_clock->now() == point);
    }
}

TEST_CASE("ClockWrapper", "[utils]")
{
    ClockWrapper clock;
    ClockTimePoint p1 = std::chrono::steady_clock::now();
    usleep(1);
    ClockTimePoint p2 = clock.now();
    std::chrono::microseconds test_duration = 
            std::chrono::duration_cast<std::chrono::microseconds>(p2 - p1);
    REQUIRE(test_duration.count() > 0);
    REQUIRE(test_duration.count() < 1000);
}


TEST_CASE("Timer", "[utils]")
{
    MockClockWrapper* mock_clock = get_mock_clock();

    Timer t;
    REQUIRE(t.dt_s() == 0);
    REQUIRE(t.dt_ms() == 0);
    REQUIRE(t.dt_us() == 0);

    mock_clock->advance_sec(0.5);
    REQUIRE(t.dt_s() == 0.5f);
    REQUIRE(t.dt_ms() == 500);
    REQUIRE(t.dt_us() == 500000);

    t.reset();
    REQUIRE(t.dt_s() == 0);
    REQUIRE(t.dt_ms() == 0);
    REQUIRE(t.dt_us() == 0);

    mock_clock->advance_ms(100);
    REQUIRE(t.dt_s() == 0.1f);
    REQUIRE(t.dt_ms() == 100);
    REQUIRE(t.dt_us() == 100000);

    t.reset();
    REQUIRE(t.dt_s() == 0);
    REQUIRE(t.dt_ms() == 0);
    REQUIRE(t.dt_us() == 0);

    mock_clock->advance_us(100);
    REQUIRE(t.dt_s() == 0.0001f);
    REQUIRE(t.dt_ms() == 0);
    REQUIRE(t.dt_us() == 100);

    t.reset();
    REQUIRE(t.dt_s() == 0);
    REQUIRE(t.dt_ms() == 0);
    REQUIRE(t.dt_us() == 0);
}