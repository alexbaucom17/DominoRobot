#include "DistanceTracker.h"

#include "utils.h"
#include <plog/Log.h> 
#include "constants.h"
#include "serial/SerialCommsFactory.h"
#include <math.h>

DistanceTracker::DistanceTracker()
: current_distance_pose_(0,0,0),
  distance_buffers_(),
  running_(false),
  measurement_time_averager_(10),
  serial_to_arduino_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(ARDUINO_USB)),
  forward_left_id_(cfg.lookup("distance.mapping.forward_left")),
  forward_right_id_(cfg.lookup("distance.mapping.forward_right")),
  angled_left_id_(cfg.lookup("distance.mapping.angled_left")),
  angled_right_id_(cfg.lookup("distance.mapping.angled_right")),
  angle_from_forward_radians_(static_cast<float>(cfg.lookup("distance.dimensions.angle_from_forward_degrees"))*M_PI/180.0),
  left_fwd_offset_(cfg.lookup("distance.dimensions.left_fwd_offset")),               
  right_fwd_offset_(cfg.lookup("distance.dimensions.right_fwd_offset")),   
  left_angle_offset_(cfg.lookup("distance.dimensions.left_angle_offset")),   
  right_angle_offset_(cfg.lookup("distance.dimensions.right_angle_offset")),   
  num_sensors_(cfg.lookup("distance.num_sensors"))
{
    for (uint i = 0; i < num_sensors_; i++)
    {
        distance_buffers_.push_back(CircularBuffer<float>(cfg.lookup("distance.samples_to_average")));
    }
}

void DistanceTracker::start()
{
    running_ = true;
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("start");
        PLOGI << "Distance measurement starting";
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
        std::vector<float> new_measurement = getMeasurement();
        if (new_measurement.empty()) return;
        for (uint i =0; i < num_sensors_; i++)
        {
            distance_buffers_[i].insert(new_measurement[i]);
        }
        computePoseFromDistances();
    }
}


std::vector<float> DistanceTracker::getMeasurement()
{
    std::string msg = "";
    if (serial_to_arduino_->isConnected())
    {
        msg = serial_to_arduino_->rcv_distance();
    }
    if(msg.empty()) return {};
    std::vector<float> values = parseCommaDelimitedStringToFloat(msg);
    if(values.size() != num_sensors_) return {};

    measurement_time_averager_.mark_point();
    return values;
}

void DistanceTracker::computePoseFromDistances()
{
    // First, compute running average of all distance buffers
    std::vector<float> mean_distances;
    mean_distances.reserve(num_sensors_);
    for (uint i = 0; i < num_sensors_; i++) 
    {
        mean_distances[i] = vectorMean(distance_buffers_[i].get_contents());
    }

    // Forward distance is easy - just average the two measurements
    Point new_pose;
    new_pose.x = (mean_distances[forward_left_id_] + mean_distances[forward_right_id_]) / 2.0;

    // Side distance is a little bit harder - need to average projection of measurements and add offset from center
    float y_projection_left = mean_distances[angled_left_id_] * sin(angle_from_forward_radians_) + left_angle_offset_;
    float y_projection_right = mean_distances[angled_right_id_] * sin(angle_from_forward_radians_) + right_angle_offset_;
    new_pose.y = (y_projection_left + y_projection_right) / 2.0;

    // Angle is the hardest - need to average angle made by front and side measurements
    float x_projection_left = mean_distances[angled_left_id_] * cos(angle_from_forward_radians_);
    float x_projection_right = mean_distances[angled_right_id_] * cos(angle_from_forward_radians_);
    float fwd_angle = atan2(right_fwd_offset_ + left_fwd_offset_, mean_distances[forward_left_id_] - mean_distances[forward_right_id_]);
    float side_angle = atan2(y_projection_left-y_projection_right,x_projection_left-x_projection_right);
    new_pose.a = (fwd_angle + side_angle) / 2.0;

}