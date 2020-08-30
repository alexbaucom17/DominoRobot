#include <Catch/catch.hpp>

#include "StatusUpdater.h"

using Catch::Matchers::StartsWith;
using Catch::Matchers::EndsWith;
using Catch::Matchers::Contains;

TEST_CASE("Position", "[StatusUpdater]")
{
    StatusUpdater s;
    s.updatePosition(1,2,3);

    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.pos_x == 1);
    REQUIRE(status.pos_y == 2);
    REQUIRE(status.pos_a == 3);
}

TEST_CASE("Velocity", "[StatusUpdater]")
{
    StatusUpdater s;
    s.updateVelocity(1,2,3);

    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.vel_x == 1);
    REQUIRE(status.vel_y == 2);
    REQUIRE(status.vel_a == 3);
}

TEST_CASE("LoopTimes", "[StatusUpdater]")
{
    StatusUpdater s;
    s.updateLoopTimes(1,2);

    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.controller_loop_ms == 1);
    REQUIRE(status.position_loop_ms == 2);
}

TEST_CASE("Progress", "[StatusUpdater]")
{
    StatusUpdater s;

    StatusUpdater::Status status = s.getStatus();
    REQUIRE(status.in_progress == false);
    REQUIRE(s.getInProgress() == false);

    s.updateInProgress(true);

    status = s.getStatus();
    REQUIRE(status.in_progress == true);
    REQUIRE(s.getInProgress() == true);
}

TEST_CASE("JSON", "[StatusUpdater]")
{
    StatusUpdater s;
    s.updatePosition(1,2,3);
    s.updateVelocity(4,5,6);
    s.updateLoopTimes(7,8);
    s.updateInProgress(true);

    std::string json_string = s.getStatusJsonString();

    REQUIRE_THAT(json_string, StartsWith("{"));
    REQUIRE_THAT(json_string, Contains("\"pos_x\":1"));
    REQUIRE_THAT(json_string, Contains("\"pos_y\":2"));
    REQUIRE_THAT(json_string, Contains("\"pos_a\":3"));
    REQUIRE_THAT(json_string, Contains("\"vel_x\":4"));
    REQUIRE_THAT(json_string, Contains("\"vel_y\":5"));
    REQUIRE_THAT(json_string, Contains("\"vel_a\":6"));
    REQUIRE_THAT(json_string, Contains("\"controller_loop_ms\":7"));
    REQUIRE_THAT(json_string, Contains("\"position_loop_ms\":8"));
    REQUIRE_THAT(json_string, EndsWith("}"));
}
