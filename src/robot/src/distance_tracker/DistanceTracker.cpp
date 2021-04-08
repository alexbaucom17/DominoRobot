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
  kf_(3,3),
  fwd_left_id_(cfg.lookup("distance_tracker.mapping.fwd_left")),
  fwd_right_id_(cfg.lookup("distance_tracker.mapping.fwd_right")),
  side_front_id_(cfg.lookup("distance_tracker.mapping.side_front")),
  side_back_id_(cfg.lookup("distance_tracker.mapping.side_back")),
  angle_from_fwd_radians_(static_cast<float>(cfg.lookup("distance_tracker.dimensions.angle_from_fwd_degrees"))*M_PI/180.0),
  fwd_left_offset_(cfg.lookup("distance_tracker.dimensions.fwd_left_offset")),               
  fwd_right_offset_(cfg.lookup("distance_tracker.dimensions.fwd_right_offset")),   
  side_front_offset_(cfg.lookup("distance_tracker.dimensions.side_front_offset")),   
  side_back_offset_(cfg.lookup("distance_tracker.dimensions.side_back_offset")),   
  num_sensors_(cfg.lookup("distance_tracker.num_sensors"))
{
    for (uint i = 0; i < num_sensors_; i++)
    {
        distance_buffers_.push_back(CircularBuffer<float>(cfg.lookup("distance_tracker.samples_to_average")));
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
        for (uint i = 0; i < num_sensors_; i++)
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
    // PLOGI << msg;
    std::vector<float> values = parseCommaDelimitedStringToFloat(msg);
    if(values.size() != num_sensors_) return {};

    // Mark when valid measurement was received
    measurement_time_averager_.mark_point();
    
    // Convert to meters
    for(auto& v : values) v /= 1000.0;
    return values;
}


// d(1|2) is distance measurement (meters)
// o(1|2) is offset in y dimension of sensor (meters)
// Returns <distance_x, angle from y axis> in <meters, rads> of line
std::pair<float, float> distAndAngleFromPairedDistanceFront(float d1, float o1, float d2, float o2)
{
    float dx = (d1 + d2) / 2.0;
    float a = atan2(d1-d2, o1-o2);
    return {dx, a};
}

// d(1|2) is distance measurement (meters)
// o(1|2) is offset in y dimension of sensor (meters)
// a(1|2) is angle of sensor from x axis (radians)
// Returns <distance_y, angle from x axis> in <meters, rads> of line
std::pair<float, float> distAndAngleFromPairedDistanceSide(float d1, float o1, float a1, float d2, float o2, float a2)
{
    float p1x = d1*cos(a1);
    float p1y = d1*sin(a1) + o1;
    float p2x = d2*cos(a2);
    float p2y = d2*sin(a2) + o2;
    float dy = (p1y + p2y) / 2.0;
    float a = atan2(p1y-p2y, p1x-p2x);
    return {dy, a};
}

void DistanceTracker::computePoseFromDistances()
{
    // First, compute running average of all distance buffers
    std::vector<float> mean_distances;
    mean_distances.reserve(num_sensors_);
    for (const auto& buf : distance_buffers_) 
    {
        mean_distances.push_back(vectorMean(buf.get_contents()));
    }

    // Using forward measurements
    auto [x_dist, fwd_angle] = distAndAngleFromPairedDistanceFront(
        mean_distances[fwd_left_id_], fwd_left_offset_, 
        mean_distances[fwd_right_id_], fwd_right_offset_);

    auto [y_dist, side_angle] = distAndAngleFromPairedDistanceSide(
        mean_distances[side_front_id_], side_front_offset_, angle_from_fwd_radians_, 
        mean_distances[side_back_id_], side_back_offset_, angle_from_fwd_radians_);

    // Grab final measurements
    // X dist is from forward sensors
    // Y dist is from angled sensors
    // Angle is average of forward and angled measurements
    current_distance_pose_.x = x_dist;
    current_distance_pose_.y = 0; //y_dist;
    current_distance_pose_.a = 0; //(fwd_angle + side_angle)/2.0;
}

std::vector<float> DistanceTracker::getRawDistances() 
{
    if(running_)
    {
        std::vector<float> mean_distances;
        mean_distances.reserve(num_sensors_);
        for (const auto& buf : distance_buffers_) 
        {
            mean_distances.push_back(vectorMean(buf.get_contents()));
        }
        return mean_distances; 
    }
    else
    {
        return std::vector<float>(num_sensors_, 0.0);
    }
}
