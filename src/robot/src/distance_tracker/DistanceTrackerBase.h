#ifndef DistanceTrackerBase_h
#define DistanceTrackerBase_h

#include "utils.h"
#include "serial/SerialComms.h"

class DistanceTrackerBase
{

  public:
    // Start measurement loop
    virtual void start() = 0;

    // Stop measurement loop
    virtual void stop() = 0;

    // Main 'update' function that must be called regularly
    virtual void checkForMeasurement() = 0;

    // Get latest distance values
    virtual Point getDistancePose() = 0;

    // Returns bool indicating if distance measurements are running
    virtual bool isRunning() = 0;

    // How long the time delay between measurements is
    virtual float getAverageMeasurementTimeSeconds() = 0;
};

#endif //DistanceTrackerBase_h
