#include "DistanceTracker.h"

#include "utils.h"
#include <plog/Log.h> 
#include "constants.h"
#include "serial/SerialCommsFactory.h"
#include <math.h>

constexpr float bad_angle = 1000;
constexpr float bad_dist = 1000;

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
  fwd_left_offset_(cfg.lookup("distance_tracker.dimensions.fwd_left_offset")),               
  fwd_right_offset_(cfg.lookup("distance_tracker.dimensions.fwd_right_offset")),   
  side_front_offset_(cfg.lookup("distance_tracker.dimensions.side_front_offset")),   
  side_back_offset_(cfg.lookup("distance_tracker.dimensions.side_back_offset")),  
  min_fwd_dist_(cfg.lookup("distance_tracker.limits.min_fwd_dist")),
  max_fwd_dist_(cfg.lookup("distance_tracker.limits.max_fwd_dist")),
  min_side_dist_(cfg.lookup("distance_tracker.limits.min_side_dist")),
  max_side_dist_(cfg.lookup("distance_tracker.limits.max_side_dist")),
  num_sensors_(cfg.lookup("distance_tracker.num_sensors"))
{
    for (uint i = 0; i < num_sensors_; i++)
    {
        distance_buffers_.push_back(CircularBuffer<float>(cfg.lookup("distance_tracker.samples_to_average")));
    }
}

void DistanceTracker::start()
{
    if (serial_to_arduino_->isConnected())
    {
        serial_to_arduino_->send("start");
        if(!running_) PLOGI << "Distance measurement starting";
    }
    running_ = true;
    running_timer_.reset();
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

        // Need to tell the arduino to reset its safety timer every so often.
        if(running_timer_.dt_s() > 25) start();
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
// Returns <distance, angle from forward> in <meters, rads> of line
std::pair<float, float> distAndAngleFromPairedDistance(float d1, float o1, float d2, float o2, float min_limit, float max_limit)
{
    bool d1_okay = true;
    bool d2_okay = true;
    if(d1 < min_limit || d1 > max_limit) d1_okay = false;
    if(d2 < min_limit || d2 > max_limit) d2_okay = false;
    
    if(d1_okay && d2_okay)
    {
        float dx = (d1 + d2) / 2.0;
        float a = atan2(d1-d2, o1-o2);
        return {dx, a};
    } 
    else if (d1_okay) 
    {
        return {d1,bad_angle};
    }
    else if (d2_okay)
    {
        return {d2, bad_angle};
    }
    else
    {
        return {bad_dist, bad_angle};
    }
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

    // Now compute distance and angles from US pairs
    auto [x_dist, fwd_angle] = distAndAngleFromPairedDistance(
        mean_distances[fwd_left_id_], fwd_left_offset_, 
        mean_distances[fwd_right_id_], fwd_right_offset_, min_fwd_dist_, max_fwd_dist_);
    auto [y_dist, side_angle] = distAndAngleFromPairedDistance(
        mean_distances[side_front_id_], side_front_offset_, 
        mean_distances[side_back_id_], side_back_offset_, min_side_dist_, max_side_dist_);

    if (x_dist == bad_dist)
    {
        PLOGW.printf("Invalid x distance. Raw values left: %.3f right: %.3f", mean_distances[fwd_left_id_], mean_distances[fwd_right_id_]);
        x_dist = current_distance_pose_.x;
    }
    if (y_dist == bad_dist)
    {
        PLOGW.printf("Invalid y distance. Raw values front: %.3f back: %.3f", mean_distances[side_front_id_], mean_distances[side_back_id_]);
        y_dist = current_distance_pose_.y;
    }

    // Grab final measurements
    // X dist is from forward sensors
    // Y dist is from side sensors
    // Angle is just from side sensors (more reliable)
    // If forward sensors can be made more reliable, angle can be averaged
    current_distance_pose_.x = x_dist;
    current_distance_pose_.y = y_dist;
    current_distance_pose_.a = side_angle;
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
