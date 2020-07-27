#ifndef SocketWrapper_h
#define SocketWrapper_h

#include <queue>
#include <thread>

#define BUFFER_SIZE 1024

// TODO: Rename this class to SocketMultiThreadWrapper
class SocketWrapper
{

  public:
    SocketWrapper();
    std::string getData();
    void sendData(std::string data);
    bool dataAvailableToRead();

  private:
    
    void socket_loop();

    std::queue<char> data_buffer;
    std::queue<char> send_buffer;
    std::thread run_thread;

};



#endif