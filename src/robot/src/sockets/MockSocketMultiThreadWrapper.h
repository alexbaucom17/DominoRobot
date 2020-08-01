#ifndef MockSocketMultiThreadWrapper_h
#define MockSocketMultiThreadWrapper_h

#include <string>
#include <queue>

#include "SocketMultiThreadWrapperBase.h"

class MockSocketMultiThreadWrapper : public SocketMultiThreadWrapperBase
{
  public:
    MockSocketMultiThreadWrapper();

    std::string getData();
    void sendData(std::string data);
    bool dataAvailableToRead();

    void add_mock_data(std::string data);

  private:
    std::queue<std::string> data_;
};

#endif