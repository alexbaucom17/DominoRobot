#ifndef MockSocketMultiThreadWrapper_h
#define MockSocketMultiThreadWrapper_h

#include <string>
#include <queue>
#include <chrono>

#include "SocketMultiThreadWrapperBase.h"

class MockSocketMultiThreadWrapper : public SocketMultiThreadWrapperBase
{
  public:
    MockSocketMultiThreadWrapper();

    std::string getData();
    void sendData(std::string data);
    bool dataAvailableToRead();

    void add_mock_data(std::string data);
    std::string getMockData();
    void sendMockData(std::string data);

    void purge_data();

  private:
    std::queue<std::string> send_data_;
    std::queue<std::string> rcv_data_;
    int ms_until_next_command_;
    std::chrono::time_point<std::chrono::steady_clock> prev_time_;

};

#endif