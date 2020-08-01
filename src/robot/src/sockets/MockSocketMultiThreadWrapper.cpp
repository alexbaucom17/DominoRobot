#include "MockSocketMultiThreadWrapper.h"

#include <plog/Log.h>

MockSocketMultiThreadWrapper::MockSocketMultiThreadWrapper()
: SocketMultiThreadWrapperBase(),
  data_()
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
    return outdata;
}

void MockSocketMultiThreadWrapper::sendData(std::string data)
{
    (void) data; // Ignore unused warning
}

bool MockSocketMultiThreadWrapper::dataAvailableToRead()
{
    return !data_.empty();
}