#include <Catch/catch.hpp>

#include "robot.h"

#include "test-utils.h"
#include <unistd.h>

// This won't work properly until I get proper clock control mocked out

// TEST_CASE("Robot move", "[Robot]")
// {
//     std::string msg = "<{'type':'move','data':{'x':0.5,'y':0.4,'a':0.3}}>";
//     libconfig::Setting& fake_perfect_motion = cfg.lookup("motion.fake_perfect_motion");
//     fake_perfect_motion = true; 
    
//     MockSocketMultiThreadWrapper* mock_socket = build_and_get_mock_socket();
//     Robot r = Robot();

//     mock_socket->sendMockData(msg);
//     usleep(100);

//     r.runOnce();
//     CHECK(r.getCurrentCommand() == COMMAND::MOVE);

//     for (int i = 0; i < 100; i++) 
//     {
//         r.runOnce();
//         usleep(100);
//     }

//     StatusUpdater::Status status = r.getStatus();
//     REQUIRE(status.pos_x == Approx(0.5).margin(0.0005));
//     REQUIRE(status.pos_y == Approx(0.4).margin(0.0005));
//     REQUIRE(status.pos_a == Approx(0.3).margin(0.0005));

//     // Have to reset at the end
//     fake_perfect_motion = false;
// }