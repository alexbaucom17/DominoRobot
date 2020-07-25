#ifndef SocketWrapper_h
#define SocketWrapper_h

#include <kissnet/kissnet.hpp>

namespace kn = kissnet;

class SocketWrapper
{

  public:
    SocketWrapper();
    void run();
    std::string getData();
    void sendData(std::string data);

  private:
    
    void socket_loop();

    kn::tcp_socket socket;
    kn::port_t port = 1234;
    std::vector<std::byte> data_buffer;
    std::mutex read_mutex;
    std::mutex send_mutex;
    kn::buffer<1024> send_buffer;
    int lentgh_to_send;

};



#endif