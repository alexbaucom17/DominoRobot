#ifndef DistanceTrackerMock_h
#define DistanceTrackerMock_h

#include "utils.h"
#include "serial/SerialComms.h"
#include "DistanceTrackerBase.h"

class DistanceTrackerMock : public DistanceTrackerBase
{

  public:
    // Start measurement loop
    void start() override {running_ = true;};

    // Stop measurement loop
    void stop() override { running_ = false; };

    // Main 'update' function that must be called regularly
    void checkForMeasurement() override {};

    // Get latest distance values
    float getDistance() override {return mock_distance_;};

    // Returns bool indicating if distance measurements are running
    bool isRunning() override {return running_;};

    void setMockDistance(float distance) {mock_distance_ = distance;};

  private:

    bool running_;
    float mock_distance_;

};

#endif //DistanceTrackerMock_h
