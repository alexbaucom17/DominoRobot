#ifndef RobotControllerModeDistance_h
#define RobotControllerModeDistance_h

#include "RobotControllerModeBase.h"
#include "SmoothTrajectoryGenerator.h"
#include "utils.h"
#include "Distance.h"

class RobotControllerModeDistance : public RobotControllerModeBase
{

  public:

    RobotControllerModeDistance(bool fake_perfect_motion, Distance& distance_tracker);

    bool startMove(Point goal_distance);

    virtual Velocity computeTargetVelocity(Point current_position, Velocity current_velocity) override;

    virtual bool checkForMoveComplete(Point current_position, Velocity current_velocity) override;

  protected:

    SmoothTrajectoryGenerator traj_gen_; 
    Point goal_distance_;
    Point current_distance_;
    PVTPoint current_target_;
    Distance& distance_tracker_;

    TrajectoryTolerances distance_tolerances_;

    PositionController x_controller_;
    PositionController y_controller_;
    PositionController a_controller_;    

};

#endif //RobotControllerModeDistance_h