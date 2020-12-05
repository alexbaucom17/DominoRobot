#ifndef MockSocketMultiThreadWrapper_h
#define MockSocketMultiThreadWrapper_h

#include <string>
#include <queue>

#include "SocketMultiThreadWrapperBase.h"
#include "utils.h"

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
    void set_send_immediate(bool send_immediate) {send_immediate_ = send_immediate;};

  private:
    std::queue<std::string> send_data_;
    std::queue<std::string> rcv_data_;
    int ms_until_next_command_;
    bool send_immediate_;
    Timer timer_;

};

#endif