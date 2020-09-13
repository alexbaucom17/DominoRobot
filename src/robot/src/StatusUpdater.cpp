#include "StatusUpdater.h"

#define VARIANCE_REF_XY 1     // 1 m
#define VARIANCE_REF_A  1.57  // 90 degrees

StatusUpdater::StatusUpdater() :
  currentStatus_()
{
}

std::string StatusUpdater::getStatusJsonString() 
{
    // Return the status string
    return currentStatus_.toJsonString();
}

void StatusUpdater::updatePosition(float x, float y, float a)
{
    currentStatus_.pos_x = x;
    currentStatus_.pos_y = y;
    currentStatus_.pos_a = a;
}

void StatusUpdater::updateVelocity(float vx, float vy, float va)
{
    currentStatus_.vel_x = vx;
    currentStatus_.vel_y = vy;
    currentStatus_.vel_a = va;
}

void StatusUpdater::updateInProgress(bool in_progress)
{
  currentStatus_.in_progress = in_progress;
}

void StatusUpdater::updateLoopTimes(int controller_loop_ms, int position_loop_ms)
{
    currentStatus_.controller_loop_ms = controller_loop_ms;
    currentStatus_.position_loop_ms = position_loop_ms;
}

void StatusUpdater::updatePositionConfidence(float cx, float cy, float ca)
{
  // Inputs are from the covariance matrix, using those values to estimate a fractional 'confidence'
  // in our positioning relative to some reference amount. This isn't any official measurement, just an estimate to use for debugging

  // Compute inverse confidence. As long as the variance isn't larger than the reference values
  // these will be between 0-1 with 0 being more confident in the positioning
  float icx = cx/VARIANCE_REF_XY;
  float icy = cy/VARIANCE_REF_XY;
  float ica = ca/VARIANCE_REF_A;

  // Flip and scale to uint8 value for transmission
  currentStatus_.confidence_x = static_cast<uint8_t>((1-icx)*255);
  currentStatus_.confidence_y = static_cast<uint8_t>((1-icy)*255);
  currentStatus_.confidence_a = static_cast<uint8_t>((1-ica)*255);
  
}

void StatusUpdater::update_motor_driver_connected(bool connected)
{
  currentStatus_.motor_driver_connected = connected;
}

void StatusUpdater::update_lifter_driver_connected(bool connected)
{
  currentStatus_.lifter_driver_connected = connected;
}
