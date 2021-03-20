#ifndef RobotControllerModePosition_h
#define RobotControllerModePosition_h

#include "RobotControllerModeBase.h"
#include "SmoothTrajectoryGenerator.h"
#include "utils.h"

class RobotControllerModePosition : public RobotControllerModeBase
{

  public:

    RobotControllerModePosition(bool fake_perfect_motion);

    bool startMove(Point current_position, Point target_position, bool fine_mode);

    virtual Velocity computeTargetVelocity(Point current_position, Velocity current_velocity) override;

    virtual bool checkForMoveComplete(Point current_position, Velocity current_velocity) override;

  protected:

    SmoothTrajectoryGenerator traj_gen_; 
    bool fine_mode_;
    Point goal_pos_;
    PVTPoint current_target_;

    TrajectoryTolerances coarse_tolerances_;
    TrajectoryTolerances fine_tolerances_;

    PositionController x_controller_;
    PositionController y_controller_;
    PositionController a_controller_;
    

};

#endif //RobotControllerModePosition_h