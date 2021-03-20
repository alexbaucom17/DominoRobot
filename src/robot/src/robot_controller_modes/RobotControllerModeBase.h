#ifndef RobotControllerModeBase_h
#define RobotControllerModeBase_h

#include "SmoothTrajectoryGenerator.h"
#include "utils.h"

class RobotControllerModeBase
{

  public:

    RobotControllerModeBase(bool fake_perfect_motion);

    virtual Velocity computeTargetVelocity(Point current_position, Velocity current_velocity, bool log_this_cycle) = 0;

    virtual bool checkForMoveComplete(Point current_position, Velocity current_velocity) = 0;

  protected:

    struct TrajectoryTolerances
    {
        float trans_pos_err;
        float ang_pos_err;
        float trans_vel_err;
        float ang_vel_err;
    };

    void startMove();

    Timer move_start_timer_;
    Timer loop_timer_;
    bool move_running_; 
    bool fake_perfect_motion_;

};

#endif //RobotControllerModeBase_h
