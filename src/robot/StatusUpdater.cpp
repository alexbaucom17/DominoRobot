#include "StatusUpdater.h"

StatusUpdater::StatusUpdater()
{
}

String StatusUpdater::getStatusJsonString() const
{
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

void StatusUpdater::updateFrequencies(float controller_freq, float position_freq)
{
    currentStatus_.controller_freq = controller_freq;
    currentStatus_.position_freq = position_freq;
}

void StatusUpdater::update_task(String cur_task)
{
    currentStatus_.current_task = cur_task;
}

void StatusUpdater::addNote(String note)
{
    currentStatus_.notes.push_back(note);
}