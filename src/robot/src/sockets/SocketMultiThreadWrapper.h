#ifndef SocketMultiThreadWrapper_h
#define SocketMultiThreadWrapper_h

#include <queue>
#include <thread>
#include "SocketMultiThreadWrapperBase.h"

#define BUFFER_SIZE 1024

class SocketMultiThreadWrapper : public SocketMultiThreadWrapperBase
{

  public:
    SocketMultiThreadWrapper();
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