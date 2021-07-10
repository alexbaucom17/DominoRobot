#ifndef RobotControllerModeStopFast_h
#define RobotControllerModeStopFast_h

#include "RobotControllerModeBase.h"
#include "SmoothTrajectoryGenerator.h"
#include "utils.h"

class RobotControllerModeStopFast : public RobotControllerModeBase
{

  public:

    RobotControllerModeStopFast(bool fake_perfect_motion);

    void startMove(Point current_position, Velocity current_velocity);

    virtual Velocity computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle) override;

    virtual bool checkForMoveComplete(Point current_position, Velocity current_velocity) override;

  protected:

    PVTPoint current_target_;
    TrajectoryTolerances fine_tolerances_;
    std::vector<float> max_decel_; 
    std::vector<float> current_decel_;
    std::vector<int> initial_vel_sign_;  

    PositionController x_controller_;
    PositionController y_controller_;
    PositionController a_controller_;
    

};

#endif //RobotControllerModePosition_h