#include "SmoothTrajectoryGenerator.h"
#include <plog/Log.h>
#include "constants.h"


SmoothTrajectoryGenerator::SmoothTrajectoryGenerator()
  : currentTrajectory_()
{
}

PVTPoint SmoothTrajectoryGenerator::lookup(float time)
{
    return PVTPoint();
}


void SmoothTrajectoryGenerator::generatePointToPointTrajectory(Point initialPoint, Point targetPoint, bool fineMode)
{    
    // Print to logs
    PLOGI.printf("Generating trajectory");
    PLOGI.printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGI.printf("Target point: %s", targetPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Generating trajectory");
    PLOGD_(MOTION_LOG_ID).printf("Starting point: %s", initialPoint.toString().c_str());
    PLOGD_(MOTION_LOG_ID).printf("Target point: %s", targetPoint.toString().c_str());

    MotionPlanningProblem mpp = buildMotionPlanningProblem(initialPoint, targetPoint, fineMode);
    currentTrajectory_ = solve(mpp);
}

void SmoothTrajectoryGenerator::generateConstVelTrajectory(Point initialPoint, Velocity velocity, float moveTime, bool fineMode)
{
  // TODO - impliment constant velocity version
}

MotionPlanningProblem SmoothTrajectoryGenerator::buildMotionPlanningProblem(Point initialPoint, Point targetPoint, bool fineMode)
{
    MotionPlanningProblem mpp;
    mpp.initialPoint_ = initialPoint;
    mpp.targetPoint_ = targetPoint;

    DynamicLimits translationalLimits;
    DynamicLimits rotationalLimits;
    if(fineMode)
    {
        translationalLimits = {MAX_TRANS_SPEED_FINE, MAX_TRANS_ACC_FINE, MAX_TRANS_JERK_FINE};
        rotationalLimits = {MAX_ROT_SPEED_FINE, MAX_ROT_ACC_FINE, MAX_ROT_JERK_FINE};
    }
    else
    {
        translationalLimits = {MAX_TRANS_SPEED_COARSE, MAX_TRANS_ACC_COARSE, MAX_TRANS_JERK_COARSE};
        rotationalLimits = {MAX_ROT_SPEED_COARSE, MAX_ROT_ACC_COARSE, MAX_ROT_JERK_COARSE};
    }

    // This scaling makes sure to give some headroom for the controller to go a bit faster than the planned limits 
    // without actually violating any hard constraints
    mpp.translationalLimits_ = translationalLimits * TRAJ_MAX_FRACTION;
    mpp.rotationalLimits_ = rotationalLimits * TRAJ_MAX_FRACTION; 

    return mpp;     
}

Trajectory SmoothTrajectoryGenerator::solve(MotionPlanningProblem problem)
{
    return Trajectory();
}