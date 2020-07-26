#ifndef SocketWrapper_h
#define SocketWrapper_h

#include <kissnet/kissnet.hpp>
#include <queue>
#include <mutex>
#include <thread>


namespace kn = kissnet;

#define BUFFER_SIZE 1024

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
    std::queue<std::byte> data_buffer;
    std::mutex read_mutex;
    std::mutex send_mutex;
    kn::buffer<BUFFER_SIZE> send_buffer;
    int length_to_send;
    std::thread run_thread;

};



#endif