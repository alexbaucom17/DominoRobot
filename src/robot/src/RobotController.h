#ifndef RobotController_h
#define RobotController_h

#include <chrono>

#include "SmoothTrajectoryGenerator.h"
#include "StatusUpdater.h"
#include "serial/SerialComms.h"
#include "utils.h"

class RobotController
{
  public:

    // Constructor
    RobotController(StatusUpdater& statusUpdater);

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
    void setCartVelCommand(Velocity target_vel);
    // Update loop for motor objects
    void updateMotors();
    // Run controller calculations
    Velocity computeControl(PVTPoint cmd);
    // Check if the current trajectory is done
    bool checkForCompletedTrajectory(const PVTPoint cmd);
    // Calculate wheel odometry
    void computeOdometry();
    // Generate a command signal for the current time in the trajectory
    PVTPoint generateCommandFromTrajectory();
    // Creates stationary command for when a trajectory is not running
    PVTPoint generateStationaryCommand();
    // Sets up everything to start the trajectory running
    void startTraj();
    // Reads an incoming message from the motor driver and returns local
    // velocity if available
    std::vector<float> readMsgFromMotorDriver();

    // Member variables
    SmoothTrajectoryGenerator trajGen_;    // Trajectory generator object
    StatusUpdater& statusUpdater_;         // Reference to status updater object to input status info about the controller
    SerialCommsBase* serial_to_motor_driver_;   // Serial connection to motor driver
    std::chrono::time_point<std::chrono::steady_clock> prevControlLoopTime_;    // Previous loop time through the cartesian control loop
    std::chrono::time_point<std::chrono::steady_clock> prevOdomLoopTime_;       // Previous loop time through the odom loop
    std::chrono::time_point<std::chrono::steady_clock> trajStartTime_;          // Previous loop time when trajectory was started
    Point cartPos_;                        // Current cartesian position
    Point goalPos_;                        // Desired goal position
    Velocity cartVel_;                     // Current cartesian velocity
    bool trajRunning_;                     // If a trajectory is currently active
    bool fineMode_;                        // If fine positioning mode is enabled or not.
    bool velOnlyMode_;                     // If we are only interested in velocity and not goal position
    RateController controller_rate_;       // Rate limit controller loops
    RateController logging_rate_ ;         // Rate limit logging to file
    bool log_this_cycle_;                  // Trigger for logging this cycle
    bool fake_perfect_motion_;             // Flag used for testing to enable perfect motion without clearcore
    Velocity fake_local_cart_vel_;         // Commanded local cartesian velocity used to fake perfect motion

    struct TrajectoryTolerances
    {
        float trans_pos_err;
        float ang_pos_err;
        float trans_vel_err;
        float ang_vel_err;
    };
    TrajectoryTolerances coarse_tolerances_;
    TrajectoryTolerances fine_tolerances_;

    float mm_update_fraction_;
    float mm_update_vel_fn_slope_;
    float mm_update_vel_fn_intercept_;

};

#endif
