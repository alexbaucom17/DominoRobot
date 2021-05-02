#include "StatusUpdater.h"
#include "constants.h"

StatusUpdater::StatusUpdater() :
  currentStatus_(),
  fwd_left_id_(cfg.lookup("distance_tracker.mapping.fwd_left")),
  fwd_right_id_(cfg.lookup("distance_tracker.mapping.fwd_right")),
  side_front_id_(cfg.lookup("distance_tracker.mapping.side_front")),
  side_back_id_(cfg.lookup("distance_tracker.mapping.side_back"))
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

void StatusUpdater::updateDistanceLoopTime(int distance_loop_ms)
{
    currentStatus_.distance_loop_ms = distance_loop_ms;
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

void StatusUpdater::updateRawDistances(std::vector<float> distances)
{
  currentStatus_.dist_fl = distances[fwd_left_id_];
  currentStatus_.dist_fr = distances[fwd_right_id_];
  currentStatus_.dist_sf = distances[side_front_id_];
  currentStatus_.dist_sb = distances[side_back_id_];
}

void StatusUpdater::updateDistancePose(Point pose)
{
  currentStatus_.dist_x = pose.x;
  currentStatus_.dist_y = pose.y;
  currentStatus_.dist_a = pose.a;
}

void StatusUpdater::updateVisionControllerPose(Point pose)
{
  currentStatus_.vision_x = pose.x;
  currentStatus_.vision_y = pose.y;
  currentStatus_.vision_a = pose.a;
}
