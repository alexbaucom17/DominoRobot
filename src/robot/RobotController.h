#ifndef RobotController_h
#define RobotController_h

#include "Motor.h"
#include "TrajectoryGenerator.h"
#include <HardwareSerial.h>
#include "StatusUpdater.h"
#include "KalmanFilter.h"

class RobotController
{
  public:

    // TODO: Convert a bunch of the math here to use the new LinearAlgebra library

    // Constructor
    RobotController(HardwareSerial& debug, StatusUpdater& statusUpdater);

    // Command robot to move a specific position
    void moveToPosition(float x, float y, float a);
    
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
    void computeOdometry(unsigned long deltaMillis);

    // Member variables
    Motor motors[4];                       // Motor interface objects
    unsigned long prevMotorLoopTime_;      // Previous loop time for motor controller
    HardwareSerial& debug_;                // Serial port to write debug info to
    bool enabled_;                         // Global motor enabled flag
    TrajectoryGenerator trajGen_;          // Trajectory generator object
    Point cartPos_;                        // Current cartesian position
    Point cartVel_;                        // Current cartesian velocity
    bool trajRunning_;                     // If a trajectory is currently active
    unsigned long trajStartTime_;          // Holds milliseconds when trajecotry was started
    float errSumX_;                        // Sum of error in X dimension for integral control
    float errSumY_;                        // Sum of error in Y dimension for integral control
    float errSumA_;                        // Sum of error in A dimension for integral control
    float prevControlLoopTime_;            // Previous time through the cartesian control loop

    StatusUpdater& statusUpdater_;

    // Kalman filter stuff
    KalmanFilter kf_;

};

#endif