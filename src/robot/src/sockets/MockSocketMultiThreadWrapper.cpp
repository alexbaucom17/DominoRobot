#include "MockSocketMultiThreadWrapper.h"

#include <plog/Log.h>

MockSocketMultiThreadWrapper::MockSocketMultiThreadWrapper()
: SocketMultiThreadWrapperBase(),
  data_(),
  ms_until_next_command_(0),
  prev_time_(std::chrono::steady_clock::now())
{
}

void MockSocketMultiThreadWrapper::add_mock_data(std::string data)
{
    data_.push(data);
}

std::string MockSocketMultiThreadWrapper::getData()
{
    std::string outdata = data_.front();
    data_.pop();
    PLOGI << "Popped: " << outdata << " data left: " << data_.size();

    if(outdata[1] != '{')
    {
        ms_until_next_command_ = stoi(outdata);
        prev_time_ = std::chrono::steady_clock::now();
        PLOGI << "Waiting " << ms_until_next_command_ << " ms to send next command";
    }

    return outdata;
}

void MockSocketMultiThreadWrapper::sendData(std::string data)
{
    (void) data; // Ignore unused warning
}

bool MockSocketMultiThreadWrapper::dataAvailableToRead()
{
    int dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time_).count();
    if(dt > ms_until_next_command_)
    {
        return !data_.empty();
    }
    else
    {
        return false;
    }
}