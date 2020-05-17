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

void StatusUpdater::updateInProgress(bool in_progress)
{
  currentStatus_.in_progress = in_progress;
}
