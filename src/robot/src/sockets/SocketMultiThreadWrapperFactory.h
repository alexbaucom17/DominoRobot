#ifndef SocketMultiThreadWrapperFactory_h
#define SocketMultiThreadWrapperFactory_h

#include <memory>

#include "SocketMultiThreadWrapperBase.h"

enum class SOCKET_FACTORY_MODE
{
    STANDARD,
    MOCK,
};

class SocketMultiThreadWrapperFactory
{

  public:

    static SocketMultiThreadWrapperFactory* getFactoryInstance();

    void set_mode(SOCKET_FACTORY_MODE mode);

    void add_mock_data(std::string data);

    void build_socket();

    SocketMultiThreadWrapperBase* get_socket();

    // Delete copy and assignment constructors
    SocketMultiThreadWrapperFactory(SocketMultiThreadWrapperFactory const&) = delete;
    SocketMultiThreadWrapperFactory& operator= (SocketMultiThreadWrapperFactory const&) = delete;

  private:

    // Make standard constructor private so it can't be created
    SocketMultiThreadWrapperFactory();

    static SocketMultiThreadWrapperFactory* instance;

    SOCKET_FACTORY_MODE mode_;
    std::unique_ptr<SocketMultiThreadWrapperBase> socket_;
    

};


#endif