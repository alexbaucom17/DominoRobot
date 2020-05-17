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

void StatusUpdater::updateSensors(bool sensor_1, bool sensor_2, bool sensor_3, bool sensor_4)
{
  currentStatus_.sensor_1 = sensor_1;
  currentStatus_.sensor_2 = sensor_2;
  currentStatus_.sensor_3 = sensor_3;
  currentStatus_.sensor_4 = sensor_4;
}

void StatusUpdater::updateInProgress(bool in_progress)
{
  currentStatus_.in_progress = in_progress;
}
