#include "StatusUpdater.h"

StatusUpdater::StatusUpdater() :
  currentStatus_(),
  notes_timers_(),
  prevMillis_(millis())
{
}

String StatusUpdater::getStatusJsonString() 
{
    updateNoteTimers();

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

void StatusUpdater::updateFrequencies(float controller_freq, float position_freq)
{
    currentStatus_.controller_freq = controller_freq;
    currentStatus_.position_freq = position_freq;
}

void StatusUpdater::update_task(String cur_task)
{
    currentStatus_.current_task = cur_task;
}

void StatusUpdater::addNote(String note, unsigned int display_time)
{
    currentStatus_.notes.push_back(note);
    notes_timers_.push_back(static_cast<unsigned long>(display_time) * 1000);
}

void StatusUpdater::updateNoteTimers()
{
    // Update status notes timers to figure out if any nodes need to
    // be removed
    unsigned long curMillis = millis();
    unsigned long dt = curMillis - prevMillis_;
    prevMillis_ = curMillis;

    // Figure out which notes should be removed.
    for (int i = 0; i < notes_timers_.size();)
    {
        // Remove if more time has passed than is left on the timer
        if(notes_timers_[i] <= dt)
        {
            // This works because we add and remove values from timers and notes at the same time
            notes_timers_.erase(notes_timers_.begin() + i);
            currentStatus_.notes.erase(currentStatus_.notes.begin() + i);
        }
        // Otherwise just decrement the timer by the past time
        else
        {
            notes_timers_[i] -= dt;
            // Only increment counter when we don't remove a value
            i++;
        }   
    }
}