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

void StatusUpdater::addNote(byte key, String note, unsigned int display_time)
{
    // Using map ensures the key only gets added once.
    // If the key is already added the timer will get refreshed
    currentStatus_.notes[key] = note;
    notes_timers_[key] = static_cast<unsigned long>(display_time) * 1000;
}

void StatusUpdater::updateNoteTimers()
{
    // Update status notes timers to figure out if any nodes need to
    // be removed
    unsigned long curMillis = millis();
    unsigned long dt = curMillis - prevMillis_;
    prevMillis_ = curMillis;

    // Figure out which notes should be removed.
    auto it = notes_timers_.begin();
    while(it != notes_timers_.end())
    {
        // Remove if more time has passed than is left on the timer
        if(it->second <= dt)
        {
            // Remove note with key
            currentStatus_.notes.erase(it->first);
            // Remove timer with iterator and get next iterator
            notes_timers_.erase(it++);
        }
        // Otherwise just decrement the timer by the past time
        else
        {
            it->second -= dt;
            // Only increment counter when we don't remove a value
            ++it;
        }   
    }
}