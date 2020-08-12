#include <Catch/catch.hpp>

#include "SmoothTrajectoryGenerator.h"

TEST_CASE("Simple case - don't crash", "[trajectory]")
{
    SmoothTrajectoryGenerator stg;
    Point p1 = {0,0,0};
    Point p2 = {1,2,3};
    bool fineMode = false;

    bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
    REQUIRE(ok == true);

    PVTPoint output = stg.lookup(1.0);
    REQUIRE(output.time_ == 1.0);
}