#include "SocketMultiThreadWrapperFactory.h"
#include "SocketMultiThreadWrapper.h"
#include "MockSocketMultiThreadWrapper.h"

#include <plog/Log.h> 

SocketMultiThreadWrapperFactory* SocketMultiThreadWrapperFactory::instance = NULL;


SocketMultiThreadWrapperFactory* SocketMultiThreadWrapperFactory::getFactoryInstance()
{
    if(!instance)
    {
        instance = new SocketMultiThreadWrapperFactory;
    }
    return instance;
}

void SocketMultiThreadWrapperFactory::set_mode(SOCKET_FACTORY_MODE mode)
{
    mode_ = mode;
}

void SocketMultiThreadWrapperFactory::add_mock_data(std::string data)
{
    if(mode_ == SOCKET_FACTORY_MODE::MOCK && socket_)
    {
        // This is slightly dangerous, but should be safe as long as I don't do something silly like 
        // change the mode of the factory after building the socket
        SocketMultiThreadWrapperBase* raw_ptr = socket_.get();
        dynamic_cast<MockSocketMultiThreadWrapper*>(raw_ptr)->add_mock_data(data);
        PLOGI << "Added mock data: " << data;
    }
}

void SocketMultiThreadWrapperFactory::build_socket()
{
    if(socket_)
    {
        return;
    }

    if(mode_ == SOCKET_FACTORY_MODE::STANDARD)
    {
        socket_ = std::make_unique<SocketMultiThreadWrapper> ();
        PLOGI << "Built SocketMultiThreadWrapper";
    }
    else if (mode_ == SOCKET_FACTORY_MODE::MOCK)
    {
        socket_ = std::make_unique<MockSocketMultiThreadWrapper> ();
        PLOGI << "Built MockSocketMultiThreadWrapper";
    }
}

std::unique_ptr<SocketMultiThreadWrapperBase> SocketMultiThreadWrapperFactory::get_socket()
{
    if(!socket_)
    {
        build_socket();
    }
    return std::move(socket_);
}


// Private constructor
SocketMultiThreadWrapperFactory::SocketMultiThreadWrapperFactory()
: mode_(SOCKET_FACTORY_MODE::STANDARD)
{}
  