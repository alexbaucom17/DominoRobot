#ifndef RobotController_h
#define RobotController_h

#include <chrono>

#include "TrajectoryGenerator.h"
#include "StatusUpdater.h"
// #include "KalmanFilter.h"

class RobotController
{
  public:

    // Constructor
    RobotController(StatusUpdater& statusUpdater);

    void begin();

    // Command robot to move a specific position with low accuracy
    void moveToPosition(float x, float y, float a);

    // Command robot to move a specific position relative to current position with low accuracy
    void moveToPositionRelative(float x, float y, float a);
    
    // Command robot to move to a specific position with high accuracy
    void moveToPositionFine(float x, float y, float a);

    // Command robot to move with a constant velocity for some amount of time
    void moveConstVel(float vx , float vy, float va, float t);

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

    // Stops the currently running motion
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
    bool checkForCompletedTrajectory(const PVTPoint cmd);
    // Calculate wheel odometry
    void computeOdometry();
    //Write velocity out to controller
    void writeVelocity(float speed, int speed_pin, int dir_pin);
    // Run the trajectory calculations and generate a command signal
    void runTraj(PVTPoint* cmd);
    // Reset everything for when a trajectory is not running
    void resetTraj(PVTPoint* cmd);
    // Sets up evertyhing to start the trajectory running
    void startTraj();

    // Member variables
    std::chrono::time_point<std::chrono::steady_clock> prevControlLoopTime_;    // Previous loop time through the cartesian control loop
    std::chrono::time_point<std::chrono::steady_clock> prevOdomLoopTime_;       // Previous loop time through the odom loop
    std::chrono::time_point<std::chrono::steady_clock> trajStartTime_;          // Previous loop time when trajecotry was started
    bool enabled_;                         // Global motor enabled flag
    TrajectoryGenerator trajGen_;          // Trajectory generator object
    Point cartPos_;                        // Current cartesian position
    Point cartVel_;                        // Current cartesian velocity
    Point goalPos_;                        // Desired goal position
    bool trajRunning_;                     // If a trajectory is currently active
    float errSumX_;                        // Sum of error in X dimension for integral control
    float errSumY_;                        // Sum of error in Y dimension for integral control
    float errSumA_;                        // Sum of error in A dimension for integral control
    bool fineMode_;                        // If fine positioning mode is enabled or not.
    bool velOnlyMode_;                     // If we are only interested in velocity and not goal position
    bool predict_once;                     // Bool to make sure kalman filter gets initialized properly

    StatusUpdater& statusUpdater_;         // Reference to status updater object to input status info about the controller

    // Kalman filter stuff
    // KalmanFilter kf_;

};

#endif
