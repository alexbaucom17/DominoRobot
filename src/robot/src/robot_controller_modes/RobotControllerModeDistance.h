#ifndef RobotControllerModeDistance_h
#define RobotControllerModeDistance_h

#include "RobotControllerModeBase.h"
#include "SmoothTrajectoryGenerator.h"
#include "utils.h"
#include "distance_tracker/DistanceTrackerBase.h"

class RobotControllerModeDistance : public RobotControllerModeBase
{

  public:

    RobotControllerModeDistance(bool fake_perfect_motion);

    bool startMove(Point goal_distance);

    virtual Velocity computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle) override;

    virtual bool checkForMoveComplete(Point current_position, Velocity current_velocity) override;

  protected:

    void actuallyStartTheMove();

    SmoothTrajectoryGenerator traj_gen_; 
    Point goal_distance_;
    Point current_distance_;
    PVTPoint current_target_;
    DistanceTrackerBase* distance_tracker_;
    bool move_started_for_real_;
    float move_start_delay_sec_;
    Timer traj_done_timer_;

    TrajectoryTolerances distance_tolerances_;

    PositionController x_controller_;
    PositionController y_controller_;
    PositionController a_controller_;    

};

#endif //RobotControllerModeDistance_h