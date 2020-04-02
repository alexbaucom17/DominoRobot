#ifndef RobotController_h
#define RobotController_h

#include "TrajectoryGenerator.h"
#include <Filters.h>
#include <HardwareSerial.h>
#include "StatusUpdater.h"
#include "KalmanFilter.h"
#include <StepperDriver.h>

class RobotController
{
  public:

    // TODO: Convert a bunch of the math here to use the new LinearAlgebra library

    // Constructor
    RobotController(HardwareSerial& debug, StatusUpdater& statusUpdater);

    void begin();

    // Command robot to move a specific position with low accuracy
    void moveToPosition(float x, float y, float a);

    // Command robot to move a specific position relative to current position with low accuracy
    void moveToPositionRelative(float x, float y, float a);
    
    // Command robot to move to a specific position with high accuracy
    void moveToPositionFine(float x, float y, float a);

    // Main update loop. Should be called as fast as possible
    void update();

    // Enable all motors at once
    void enableAllMotors();

    // Disable all motors at once. This will cause motors to coast to a stop
    void disableAllMotors();

    // Provide a position reading from the MarvelMind sensors
    void inputPosition(float x, float y, float a);

    // Indicates if a trajectory is currently active
    bool isTrajectoryRunning() { return trajRunning_; };

    void estop();

  private:

    //Internal methods
    // Set the global cartesian velocity command
    void setCartVelCommand(float vx, float vy, float va);
    // Update loop for motor objects
    void updateMotors();
    // Run controller calculations
    void computeControl(PVTPoint cmd);
    // Check if the current trajectory is done
    bool checkForCompletedTrajectory(PVTPoint cmd);
    // Calculate wheel odometry
    void computeOdometry();

    // Member variables
    axis_t motors[4];                      // Motor interface objects
    unsigned long prevPositionUpdateTime_; // Previous loop millis we were provided a position observation
    unsigned long prevControlLoopTime_;    // Previous loop millis through the cartesian control loop
    unsigned long prevUpdateLoopTime_;     // Previous loop millis through the update loop
    unsigned long prevOdomLoopTime_;       // Previous loop millis through the odom loop
    HardwareSerial& debug_;                // Serial port to write debug info to
    bool enabled_;                         // Global motor enabled flag
    TrajectoryGenerator trajGen_;          // Trajectory generator object
    Point cartPos_;                        // Current cartesian position
    Point cartVel_;                        // Current cartesian velocity
    Point goalPos_;                        // Desired goal position
    bool trajRunning_;                     // If a trajectory is currently active
    unsigned long trajStartTime_;          // Holds milliseconds when trajecotry was started
    float errSumX_;                        // Sum of error in X dimension for integral control
    float errSumY_;                        // Sum of error in Y dimension for integral control
    float errSumA_;                        // Sum of error in A dimension for integral control
    bool fineMode_;                        // If fine positioning mode is enabled or not.
    bool predict_once;                     // Bool to make sure kalman filter gets initialized properly
    float motor_velocities[4];             // Track motor velocities in rad/s

    StatusUpdater& statusUpdater_;         // Reference to status updater object to input status info about the controller
    RunningStatistics controller_time_averager_;  // Handles keeping average of the controller loop timing
    RunningStatistics position_time_averager_;    // Handles keeping average of the position update timing

    // Kalman filter stuff
    KalmanFilter kf_;

};

#endif
