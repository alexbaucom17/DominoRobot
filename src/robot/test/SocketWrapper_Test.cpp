#include <Catch/catch.hpp>
#include <kissnet/kissnet.hpp>
#include <unistd.h>

#include "SocketWrapper.h"

void SimpleSocketSend(std::string data_to_send)
{
    //Create a kissnet tcp ipv4 socket
    kn::tcp_socket test_socket(kn::endpoint("localhost:8123"));
    test_socket.connect();

    //Send message
    test_socket.send(reinterpret_cast<const std::byte*>(data_to_send.c_str()), data_to_send.size());

    // Brief wait to ensure thread processed the sent message
    usleep(1000);
}

TEST_CASE("Socket test", "[socket]") 
{
    SocketWrapper s;
    std::string test_msg = "This is a test";
    for(int i = 0; i < 10; i++)
    {
        test_msg += std::to_string(i);
        SimpleSocketSend(test_msg);
        REQUIRE(s.dataAvailableToRead());
        REQUIRE(s.getData() == test_msg);
    }
}