#include <Catch/catch.hpp>
#include <unistd.h>

#include "sockets/ClientSocket.h"
#include "sockets/SocketException.h"
#include "sockets/SocketMultiThreadWrapper.h"



void SimpleSocketSend(std::string data_to_send)
{
    usleep(1000);
    
    ClientSocket test_socket ( "localhost", 8123 );

    //Send message
    test_socket << data_to_send;

    // Brief wait to ensure thread processed the sent message
    usleep(1000);
}

// TODO: Figure out why this is broken
// TEST_CASE("Socket send test", "[socket]") 
// {
    
//     SocketMultiThreadWrapper s;
//     std::string test_msg = "This is a test";
//     for(int i = 0; i < 10; i++)
//     {
//         test_msg += std::to_string(i);
//         SimpleSocketSend(test_msg);
//         REQUIRE(s.dataAvailableToRead());
//         REQUIRE(s.getData() == test_msg);
//     }   
// }

// TEST_CASE("Socket recv test", "[socket]") 
// {
    
//     SocketMultiThreadWrapper s;
//     std::string test_msg = "This is a test";
//     for(int i = 0; i < 10; i++)
//     {
//         test_msg += std::to_string(i);

//         usleep(1000);
//         ClientSocket test_socket ( "localhost", 8123 );
//         s.sendData(test_msg);

//         std::string data;
//         test_socket >> data;
//         REQUIRE(data == test_msg);
//     }   
// }