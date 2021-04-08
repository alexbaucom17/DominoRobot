#ifndef DistanceTracker_h
#define DistanceTracker_h

#include "utils.h"
#include "serial/SerialComms.h"
#include "DistanceTrackerBase.h"
#include "KalmanFilter.h"

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

    // How long the time delay between measurements is
    int getAverageMeasurementTimeMs() override {return measurement_time_averager_.get_ms();};

    std::vector<float> getRawDistances() override;

  private:

    // Handles getting measurements from serial port and parsing into number
    std::vector<float> getMeasurement();

    // Computes the current pose from the buffered distances
    void computePoseFromDistances();

    Point current_distance_pose_;
    std::vector<CircularBuffer<float>> distance_buffers_;
    bool running_;
    TimeRunningAverage measurement_time_averager_;
    SerialCommsBase* serial_to_arduino_; 
    KalmanFilter kf_;

    // Various constant parameters
    int fwd_left_id_;
    int fwd_right_id_;
    int side_front_id_;
    int side_back_id_;
    float angle_from_fwd_radians_;
    float fwd_left_offset_;                   
    float fwd_right_offset_;
    float side_front_offset_;
    float side_back_offset_;
    uint num_sensors_;

};


#endif //DistanceTracker_h