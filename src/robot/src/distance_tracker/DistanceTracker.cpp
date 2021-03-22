#include "DistanceTracker.h"

#include "utils.h"
#include <plog/Log.h> 
#include "constants.h"
#include "serial/SerialCommsFactory.h"
#include <math.h>

// TODO: Convert to proper config values
constexpr int FWD_LEFT = 0;
constexpr int FWD_RIGHT = 1;
constexpr int ANGLE_LEFT = 2;
constexpr int ANGLE_RIGHT = 3;
constexpr float ANGLE_FROM_FWD = 45 * M_PI/180.0;
constexpr int NUM_SENSORS = 4;
constexpr float LEFT_OFFSET = 0.1;
constexpr float RIGHT_OFFSET = -0.1;
constexpr int SAMPLES_TO_AVERAGE = 10;


DistanceTracker::DistanceTracker()
: current_distance_pose_(0,0,0),
  distance_buffers_(),
  running_(false),
  serial_to_arduino_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(ARDUINO_USB))
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        distance_buffers_.push_back(CircularBuffer<float>(SAMPLES_TO_AVERAGE));
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
        for (int i =0; i < NUM_SENSORS; i++)
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
    if(values.size() != NUM_SENSORS) return {};

    return values;
}

void DistanceTracker::computePoseFromDistances()
{
    // First, compute running average of all distance buffers
    std::vector<float> mean_distances;
    mean_distances.reserve(NUM_SENSORS);
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        mean_distances[i] = vectorMean(distance_buffers_[i].get_contents());
    }

    // Forward distance is easy - just average the two measurements
    Point new_pose;
    new_pose.x = (mean_distances[FWD_LEFT] + mean_distances[FWD_RIGHT]) / 2.0;

    // Side distance is a little bit harder - need to average projection of measurements and add offset from center
    float y_projection_left = mean_distances[ANGLE_LEFT] * sin(ANGLE_FROM_FWD) + LEFT_OFFSET;
    float y_projection_right = mean_distances[ANGLE_RIGHT] * sin(ANGLE_FROM_FWD) + RIGHT_OFFSET;
    new_pose.y = (y_projection_left + y_projection_right) / 2.0;

    // Angle is the hardest - need to average angle made by front and side measurements
    float x_projection_left = mean_distances[ANGLE_LEFT] * cos(ANGLE_FROM_FWD);
    float x_projection_right = mean_distances[ANGLE_RIGHT] * cos(ANGLE_FROM_FWD);
    float fwd_angle = atan2(RIGHT_OFFSET+LEFT_OFFSET,mean_distances[FWD_LEFT] - mean_distances[FWD_RIGHT]);
    float side_angle = atan2(y_projection_left-y_projection_right,x_projection_left-x_projection_right);
    new_pose.a = (fwd_angle + side_angle) / 2.0;

}