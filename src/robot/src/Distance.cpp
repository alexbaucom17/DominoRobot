#include "Distance.h"
#include <mutex>
#include "utils.h"
#include <plog/Log.h> 
#include "constants.h"
#include "serial/SerialCommsFactory.h"

std::mutex mutex;
#define HIGH 1
#define LOW 0

Distance::Distance()
: current_distance_mm_(0.0),
  distance_buffer_(20),
  running_(false),
  serial_to_arduino_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(ARDUINO_USB))
{
    run_thread_ = std::thread(&Distance::measurementLoop, this);
    run_thread_.detach();
}

void Distance::start()
{
    std::lock_guard<std::mutex> lock(mutex);
    running_ = true;
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("start");
    }
}
void Distance::stop()
{
    std::lock_guard<std::mutex> lock(mutex);
    running_ = false;
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("start");
    }
}

float Distance::getDistance()
{
    std::lock_guard<std::mutex> lock(mutex);
    return current_distance_mm_;
}

bool Distance::isRunning() 
{
    std::lock_guard<std::mutex> lock(mutex);
    return running_;
}

void Distance::measurementLoop()
{
    while(true)
    {
        while(isRunning())
        {
            float new_measurement = getMeasurement();
            if (new_measurement < 0) continue;
            distance_buffer_.insert(new_measurement);
            const std::vector<float> data = distance_buffer_.get_contents();
            float new_mean = vectorMean(data);
            std::lock_guard<std::mutex> lock(mutex);
            current_distance_mm_ = new_mean;
        }

    }
}


float Distance::getMeasurement()
{
    std::string msg = "";
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (serial_to_arduino_->isConnected())
        {
            msg = serial_to_arduino_->rcv_base();
        }
    }
    if(msg.empty()) return -1;
    std::vector<float> values = parseCommaDelimitedStringToFloat(msg);
    if(values.size() != 1) return -1;

    return values[0];
}