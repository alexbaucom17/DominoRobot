#include <Catch/catch.hpp>

#include "constants.h"

// TEST_CASE("Mock socket config disabled", "[SanityCheck]")
// {
//     bool mock_socket_enabled = cfg.lookup("mock_socket.enabled");
//     REQUIRE(mock_socket_enabled == false);
// }

TEST_CASE("Fake perfect motion disabled", "[SanityCheck]")
{
    bool fake_perfect_motion_enabled = cfg.lookup("motion.fake_perfect_motion");
    REQUIRE(fake_perfect_motion_enabled == false);
}