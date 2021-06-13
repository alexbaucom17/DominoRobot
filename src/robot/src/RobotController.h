#ifndef RobotController_h
#define RobotController_h

#include "SmoothTrajectoryGenerator.h"
#include "StatusUpdater.h"
#include "serial/SerialComms.h"
#include "utils.h"
#include "Localization.h"
#include "robot_controller_modes/RobotControllerModeBase.h"

class RobotController
{
  public:

    // Constructor
    RobotController(StatusUpdater& statusUpdater);

    // Command robot to move a specific position with low accuracy
    void moveToPosition(float x, float y, float a);

    // Command robot to move a specific position relative to current position with low accuracy
    void moveToPositionRelative(float dx_local, float dy_local, float da_local);
    
    // Command robot to move to a specific position with high accuracy
    void moveToPositionFine(float x, float y, float a);

    // Command robot to move with a constant velocity for some amount of time
    void moveConstVel(float vx , float vy, float va, float t);

    void moveWithVision(float x, float y, float a);

    // Main update loop. Should be called as fast as possible
    void update();

    // Enable all motors at once
    void enableAllMotors();

    // Disable all motors at once. This will cause motors to coast to a stop
    void disableAllMotors();

    // Provide a position reading from the MarvelMind sensors
    void inputPosition(float x, float y, float a);

    // Force the position to a specific value, bypassing localization algorithms (used for testing/debugging)
    void forceSetPosition(float x, float y, float a);

    // Indicates if a trajectory is currently active
    bool isTrajectoryRunning() { return trajRunning_; };

    // Stops the currently running motion
    void estop();

  private:

    //Internal methods
    // Set the global cartesian velocity command
    void setCartVelCommand(Velocity target_vel);
    // Update loop for motor objects
    void updateMotors();
    // Calculate wheel odometry
    void computeOdometry();
    // Sets up everything to start the trajectory running
    void startTraj();
    // Reads an incoming message from the motor driver and fills the decoded
    // velocity in the pointer, if available. Returns true if velocity is filled, false otherwise
    bool readMsgFromMotorDriver(Velocity* decodedVelocity);

    // Member variables
    StatusUpdater& statusUpdater_;         // Reference to status updater object to input status info about the controller
    SerialCommsBase* serial_to_motor_driver_;   // Serial connection to motor driver
    Localization localization_;            // Object that handles localization
    Timer prevOdomLoopTimer_;              // Timer for odom loop
    Point cartPos_;                        // Current cartesian position
    Velocity cartVel_;                     // Current cartesian velocity
    bool trajRunning_;                     // If a trajectory is currently active
    LIMITS_MODE limits_mode_;              // Which limits mode is being used.
    RateController controller_rate_;       // Rate limit controller loops
    RateController logging_rate_ ;         // Rate limit logging to file
    bool log_this_cycle_;                  // Trigger for logging this cycle
    bool fake_perfect_motion_;             // Flag used for testing to enable perfect motion without clearcore
    Velocity fake_local_cart_vel_;         // Commanded local cartesian velocity used to fake perfect motion
    const Velocity max_cart_vel_limit_;    // Maximum velocity allowed, used to limit commanded velocity

    TimeRunningAverage loop_time_averager_;        // Handles keeping average of the loop timing

    std::unique_ptr<RobotControllerModeBase> controller_mode_;

};

#endif
