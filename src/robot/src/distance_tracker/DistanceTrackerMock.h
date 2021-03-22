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

    // Get latest distance values in meters
    Point getDistancePose() override {return mock_distance_pose_;};

    // Returns bool indicating if distance measurements are running
    bool isRunning() override {return running_;};

    // Set mock distance value (meters)
    void setMockDistancePose(Point distance_pose) {mock_distance_pose_ = distance_pose;};

  private:

    bool running_;
    Point mock_distance_pose_;

};

#endif //DistanceTrackerMock_h
