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
    std::string getData();
    void sendData(std::string data);

  private:
    
    void socket_loop();

    std::queue<std::byte> data_buffer;
    kn::buffer<BUFFER_SIZE> send_buffer;
    std::mutex read_mutex;
    std::mutex send_mutex;
    int length_to_send;
    std::thread run_thread;

};



#endif