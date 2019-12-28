#include "StatusUpdater.h"

StatusUpdater::StatusUpdater() :
  currentStatus_()
{
}

String StatusUpdater::getStatusJsonString() 
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

void StatusUpdater::updateLoopTimes(int controller_loop_ms, int position_loop_ms)
{
    currentStatus_.controller_loop_ms = controller_loop_ms;
    currentStatus_.position_loop_ms = position_loop_ms;
}
