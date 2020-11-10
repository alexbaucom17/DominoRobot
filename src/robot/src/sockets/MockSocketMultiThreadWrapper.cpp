#include "MockSocketMultiThreadWrapper.h"

#include <plog/Log.h>

MockSocketMultiThreadWrapper::MockSocketMultiThreadWrapper()
: SocketMultiThreadWrapperBase(),
  send_data_(),
  rcv_data_(),
  ms_until_next_command_(-1),
  timer_()
{
}

void MockSocketMultiThreadWrapper::add_mock_data(std::string data)
{
    rcv_data_.push(data);
}

std::string MockSocketMultiThreadWrapper::getData()
{
    if (rcv_data_.empty())
    {
        return "";
    }
    
    std::string outdata = rcv_data_.front();
    rcv_data_.pop();
    PLOGI << "Popped: " << outdata << " data left: " << rcv_data_.size();

    if(outdata[1] != '{')
    {
        ms_until_next_command_ = stoi(outdata);
        timer_.reset();
        PLOGI << "Waiting " << ms_until_next_command_ << " ms to send next command";
    }

    return outdata;
}

void MockSocketMultiThreadWrapper::sendData(std::string data)
{
    send_data_.push(data);
}

bool MockSocketMultiThreadWrapper::dataAvailableToRead()
{
    if(timer_.dt_ms() > ms_until_next_command_)
    {
        return !rcv_data_.empty();
    }
    else
    {
        return false;
    }
}

std::string MockSocketMultiThreadWrapper::getMockData()
{
    if (send_data_.empty())
    {
        return "";
    }
    
    std::string outdata = send_data_.front();
    send_data_.pop();
    return outdata;
}

void MockSocketMultiThreadWrapper::sendMockData(std::string data)
{
    rcv_data_.push(data);
}

void MockSocketMultiThreadWrapper::purge_data()
{
    while(!send_data_.empty())
    {
        send_data_.pop();
    }
    while(!rcv_data_.empty())
    {
        rcv_data_.pop();
    }
}