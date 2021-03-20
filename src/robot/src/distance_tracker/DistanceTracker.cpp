#include "DistanceTracker.h"

#include "utils.h"
#include <plog/Log.h> 
#include "constants.h"
#include "serial/SerialCommsFactory.h"

DistanceTracker::DistanceTracker()
: current_distance_mm_(0.0),
  distance_buffer_(10),
  running_(false),
  serial_to_arduino_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(ARDUINO_USB))
{
}

void DistanceTracker::start()
{
    running_ = true;
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("start");
        PLOGI << "Distnace measurement starting";
    }
}

void DistanceTracker::stop()
{
    running_ = false;
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("stop");
        PLOGI << "Distance measurement stopped";
    }
}


void DistanceTracker::checkForMeasurement()
{
    if(running_)
    {
        float new_measurement = getMeasurement();
        if (new_measurement < 0) return;
        distance_buffer_.insert(new_measurement);
        const std::vector<float> data = distance_buffer_.get_contents();
        float new_mean = vectorMean(data);
        current_distance_mm_ = new_mean;
    }
}


float DistanceTracker::getMeasurement()
{
    std::string msg = "";
    if (serial_to_arduino_->isConnected())
    {
        msg = serial_to_arduino_->rcv_distance();
    }
    if(msg.empty()) return -1;
    std::vector<float> values = parseCommaDelimitedStringToFloat(msg);
    if(values.size() != 1) return -1;

    return values[0];
}