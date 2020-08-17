#ifndef SocketMultiThreadWrapperBase_h
#define SocketMultiThreadWrapperBase_h

#include <string>

class SocketMultiThreadWrapperBase
{
  public:
    virtual ~SocketMultiThreadWrapperBase() {};
    virtual std::string getData() = 0;
    virtual void sendData(std::string data) = 0;
    virtual bool dataAvailableToRead() = 0;
};



#endif