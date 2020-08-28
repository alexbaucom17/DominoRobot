#include <Catch/catch.hpp>

#include "RobotController.h"
#include "StatusUpdater.h"

// TODO: Add more tests

TEST_CASE("RobotController class", "[RobotController]")
{
    StatusUpdater s;
    RobotController r = RobotController(s);

    r.moveToPosition(1,1,1);
    r.update();

    REQUIRE(r.isTrajectoryRunning() == true);    
}