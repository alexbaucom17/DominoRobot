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
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();

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
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();

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
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();

    SECTION("Partial buffer")
    {

        for(int i = 0; i < 6; i++)
        {
            T.mark_point();
            mock_clock->advance_us(sleep_time_us);
            sleep_time_us += 2000;
        }
        REQUIRE(T.get_ms() == 6);

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
    SafeConfigModifier<bool> config_modifier("motion.rate_always_ready", false);
    SECTION("Slow")
    {
        MockClockWrapper* mock_clock = get_mock_clock_and_reset();
        RateController r(1);
        REQUIRE(r.ready() == false);
        mock_clock->advance_sec(0.5);
        REQUIRE(r.ready() == false);
        mock_clock->advance_sec(0.55);
        REQUIRE(r.ready() == true);
        REQUIRE(r.ready() == false);
    }
    SECTION("Fast")
    {
        MockClockWrapper* mock_clock = get_mock_clock_and_reset();
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
}


TEST_CASE("MockClockWrapper", "[utils]")
{
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
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
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();

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

TEST_CASE("CircularBuffer", "[utils]")
{
    SECTION("CheckFull")
    {
        CircularBuffer<int> buf(5);

        buf.insert(1);
        buf.insert(2);
        buf.insert(3);
        buf.insert(4);
        REQUIRE(buf.isFull() == false);

        buf.insert(1);
        REQUIRE(buf.isFull() == true);

        buf.insert(2);
        REQUIRE(buf.isFull() == true);

        buf.clear();
        REQUIRE(buf.isFull() == false);
    }
    SECTION("CheckContents")
    {
        CircularBuffer<int> buf(5);

        buf.insert(1);
        buf.insert(2);
        buf.insert(3);
        buf.insert(4);
        REQUIRE(buf.get_contents() == std::vector<int>{1,2,3,4});

        buf.insert(5);
        REQUIRE(buf.get_contents() == std::vector<int>{1,2,3,4,5});

        buf.insert(6);
        REQUIRE(buf.get_contents() == std::vector<int>{2,3,4,5,6});

        buf.clear();
        REQUIRE(buf.get_contents() == std::vector<int>{});

    }
    
}


TEST_CASE("PositionController", "[utils]")
{

    SECTION("P only")
    {
        PositionController::Gains gains {1,0,0};
        PositionController controller(gains);

        float target_pos = 1.0;
        float actual_pos = 0.0;
        float target_vel = 0.0;
        float actual_vel = 0.0;
        float dt = 0.1;
        float cmd_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);

        REQUIRE(cmd_vel == 1.0);
    }

    SECTION("PD")
    {
        PositionController::Gains gains {1,0,0.5};
        PositionController controller(gains);

        float target_pos = 1.0;
        float actual_pos = 0.0;
        float target_vel = 1.0;
        float actual_vel = 0.0;
        float dt = 0.1;
        float cmd_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);

        REQUIRE(cmd_vel == 2.5);
    }

    SECTION("PID")
    {
        PositionController::Gains gains {1,1,0.5};
        PositionController controller(gains);

        float target_pos = 1.0;
        float actual_pos = 0.0;
        float target_vel = 1.0;
        float actual_vel = 0.0;
        float dt = 0.1;
        float cmd_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);

        
        // Compute again to get additive I gain
        cmd_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);
        REQUIRE(cmd_vel == 2.7f);

        // Reset gain
        controller.reset();
        cmd_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);
        REQUIRE(cmd_vel == 2.6f);
    }

    SECTION("PID step response no noise")
    {
        PositionController::Gains gains {2,1,0.01};
        PositionController controller(gains);

        float target_pos = 1.0;
        float actual_pos = 0.0;
        float target_vel = 0.0;
        float actual_vel = 0.0;
        float dt = 0.1;

        for (int i = 0; i < 100; i++) 
        {
            actual_vel = controller.compute(target_pos, actual_pos, target_vel, actual_vel, dt);
            actual_pos += actual_vel*dt;
        }

        REQUIRE(actual_vel == Approx(target_vel).margin(0.001));
        REQUIRE(actual_pos == Approx(target_pos).margin(0.001));
    }

}