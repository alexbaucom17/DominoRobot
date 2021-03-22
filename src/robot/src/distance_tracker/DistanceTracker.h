#ifndef DistanceTracker_h
#define DistanceTracker_h

#include "utils.h"
#include "serial/SerialComms.h"
#include "DistanceTrackerBase.h"

class DistanceTracker : public DistanceTrackerBase
{

  public:
    DistanceTracker();

    // Start measurement loop
    void start() override;

    // Stop measurement loop
    void stop() override;

    // Main 'update' function that must be called regularly
    void checkForMeasurement() override;

    // Get latest distance values in meters
    Point getDistancePose() override {return current_distance_pose_;};

    // Returns bool indicating if distance measurements are running
    bool isRunning() override { return running_;};

  private:

    // Handles getting measurements from serial port and parsing into number
    std::vector<float> getMeasurement();

    // Computes the current pose from the buffered distances
    void computePoseFromDistances();

    Point current_distance_pose_;
    std::vector<CircularBuffer<float>> distance_buffers_;
    bool running_;

    SerialCommsBase* serial_to_arduino_; 
};


#endif //DistanceTracker_h