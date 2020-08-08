#include "QuinticTrajectoryGenerator.h"


QuinticTrajectoryGenerator::QuinticTrajectoryGenerator()
  : currentTrajectory_()
{
}

PVTPoint QuinticTrajectoryGenerator::lookup(float time)
{

}


void QuinticTrajectoryGenerator::generatePointToPointTrajectory(const Point& initialPoint, const Point& targetPoint, const DynamicLimits& limits)
{
    // Compute how far we need to go
    Point deltaPoint = targetPoint - initialPoint;
    
    // Print to logs
    PLOGI.printf("Generating trajectory");
    PLOGI.printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGI.printf("Target point: %s", targetPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Generating trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target point: %s", targetPoint.toString().c_str());

    // Scale max speeds and accelerations for trajectory generation
    DynamicLimits scaled_limits = TRAJ_MAX_FRACTION * limits;

    // Determine for translational and rotational trajectories if we can reach the max velocity
    bool reach_max_vel_for_trans = canReachMaxVelDuringTraj(deltaPoint.head(2).norm(), scaled_limits.max_vel_trans_, scaled_limits.max_acc_trans_);
    bool reach_max_vel_for_rot = canReachMaxVelDuringTraj(abs(deltaPoint.tail(1)), scaled_limits.max_vel_rot_, scaled_limits.max_acc_rot_);


}

void QuinticTrajectoryGenerator::generateConstVelTrajectory(const Point& initialPoint, const Velocity& velocity, const float moveTime, const DynamicLimits& limits)
{

}

bool canReachMaxVelDuringTraj(float dist, float max_vel, float max_acc)
{
    float timeForConstVelTrans = TRAJ_MAX_TRANS_SPEED / TRAJ_MAX_TRANS_ACC;
    float posForConstVelTrans = 0.5 * TRAJ_MAX_TRANS_ACC * timeForConstVelTrans * timeForConstVelTrans;
    float timeForConstVelRot = TRAJ_MAX_ROT_SPEED / TRAJ_MAX_ROT_ACC;
    float posForConstVelRot = 0.5 * TRAJ_MAX_ROT_ACC * timeForConstVelRot * timeForConstVelRot;
}

