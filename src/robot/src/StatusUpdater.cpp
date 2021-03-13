#include "StatusUpdater.h"

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

void StatusUpdater::updateControlLoopTime(int controller_loop_ms)
{
    currentStatus_.controller_loop_ms = controller_loop_ms;
}

void StatusUpdater::updatePositionLoopTime(int position_loop_ms)
{
    currentStatus_.position_loop_ms = position_loop_ms;
}

void StatusUpdater::updateLocalizationMetrics(LocalizationMetrics localization_metrics)
{
    currentStatus_.localization_metrics = localization_metrics;
}

void StatusUpdater::update_motor_driver_connected(bool connected)
{
  currentStatus_.motor_driver_connected = connected;
}

void StatusUpdater::update_lifter_driver_connected(bool connected)
{
  currentStatus_.lifter_driver_connected = connected;
}
