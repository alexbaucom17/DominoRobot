#include "TrajectoryGenerator.h"
#include <math.h>
#include "utils.h"



TrajectoryGenerator::TrajectoryGenerator()
  : currentTraj_()
{
}

void TrajectoryGenerator::generate(const Point& initialPoint, const Point& targetPoint, const DynamicLimits& limits)
{
    Point deltaPoint;
    deltaPoint.x_ = targetPoint.x_ - initialPoint.x_;
    deltaPoint.y_ = targetPoint.y_ - initialPoint.y_;
    deltaPoint.a_ = targetPoint.a_ - initialPoint.a_;

    // Scale max speeds and accelerations for trajectory generation
    float TRAJ_MAX_TRANS_SPEED = TRAJ_MAX_FRACTION * limits.max_trans_vel_;
    float TRAJ_MAX_TRANS_ACC = TRAJ_MAX_FRACTION * limits.max_trans_acc_;
    float TRAJ_MAX_ROT_SPEED = TRAJ_MAX_FRACTION * limits.max_rot_vel_;
    float TRAJ_MAX_ROT_ACC = TRAJ_MAX_FRACTION * limits.max_rot_acc_;

    // Pre-compute some useful values
    float timeForConstVelTrans = TRAJ_MAX_TRANS_SPEED / TRAJ_MAX_TRANS_ACC;
    float posForConstVelTrans = 0.5 * TRAJ_MAX_TRANS_ACC * timeForConstVelTrans * timeForConstVelTrans;
    float timeForConstVelRot = TRAJ_MAX_ROT_SPEED / TRAJ_MAX_ROT_ACC;
    float posForConstVelRot = 0.5 * TRAJ_MAX_ROT_ACC * timeForConstVelRot * timeForConstVelRot;

    #ifdef PRINT_DEBUG
    PLOGI.printf("Generating trajectory");
    PLOGI.printf("Starting point:");
    initialPoint.print();
    PLOGI.printf("");
    PLOGI.printf("Target point: ");
    targetPoint.print();
    PLOGI.printf("");
    #endif

    // Compute X trajectory
    if(fabs(deltaPoint.x_) < 2*posForConstVelTrans)
    {
        currentTraj_.xtraj_ = generate_triangle_1D(initialPoint.x_, targetPoint.x_, TRAJ_MAX_TRANS_SPEED, TRAJ_MAX_TRANS_ACC);
    }
    else
    {
        currentTraj_.xtraj_ = generate_trapazoid_1D(initialPoint.x_, targetPoint.x_, TRAJ_MAX_TRANS_SPEED, TRAJ_MAX_TRANS_ACC);
    }

    // Compute y trajectory
    if(fabs(deltaPoint.y_) < 2*posForConstVelTrans)
    {
        currentTraj_.ytraj_ = generate_triangle_1D(initialPoint.y_, targetPoint.y_, TRAJ_MAX_TRANS_SPEED, TRAJ_MAX_TRANS_ACC);
    }
    else
    {
        currentTraj_.ytraj_ = generate_trapazoid_1D(initialPoint.y_, targetPoint.y_, TRAJ_MAX_TRANS_SPEED, TRAJ_MAX_TRANS_ACC);
    }

    // Compute angle trajectory
    if(fabs(deltaPoint.a_) < 2*posForConstVelRot)
    {
        currentTraj_.atraj_ = generate_triangle_1D(initialPoint.a_, targetPoint.a_, TRAJ_MAX_ROT_SPEED, TRAJ_MAX_ROT_ACC);
    }
    else
    {
        currentTraj_.atraj_ = generate_trapazoid_1D(initialPoint.a_, targetPoint.a_, TRAJ_MAX_ROT_SPEED, TRAJ_MAX_ROT_ACC);
    }

    #ifdef PRINT_DEBUG
    currentTraj_.print();
    #endif
    
}

void TrajectoryGenerator::generateConstVel(const Point& initialPoint,
                                           const float vx,
                                           const float vy,
                                           const float va,
                                           const float t,
                                           const DynamicLimits& limits)
{

    // Scale max speeds and accelerations for trajectory generation
    float TRAJ_MAX_TRANS_SPEED = (fabs(vx) > fabs(vy)) ? fabs(vx) : fabs(vy);
    float TRAJ_MAX_TRANS_ACC = TRAJ_MAX_FRACTION * limits.max_trans_acc_;
    float TRAJ_MAX_ROT_SPEED = fabs(va);
    float TRAJ_MAX_ROT_ACC = TRAJ_MAX_FRACTION * limits.max_rot_acc_;

    // Pre-compute some useful values
    float timeForConstVelTrans = TRAJ_MAX_TRANS_SPEED / TRAJ_MAX_TRANS_ACC;
    float posForConstVelTrans = 0.5 * TRAJ_MAX_TRANS_ACC * timeForConstVelTrans * timeForConstVelTrans;
    float timeForConstVelRot = TRAJ_MAX_ROT_SPEED / TRAJ_MAX_ROT_ACC;
    float posForConstVelRot = 0.5 * TRAJ_MAX_ROT_ACC * timeForConstVelRot * timeForConstVelRot;

    // NOTE: Not checking angular component right now, can add later if needed
    if(t < 2*timeForConstVelTrans)
    {
        // If the time given is too short to actually reach constant vel, just estimate a target point so that 
        // we actually do something, and then print out a big warning
        #ifdef PRINT_DEBUG
        PLOGI.printf("WARNING: SPECIFIED TRAJECTORY TIME ");
        PLOGI.printf(t);
        PLOGI.printf(" LESS THAN REQUIRED TIME ");
        PLOGI.printf(2*timeForConstVelTrans);
        #endif

        Point targetPoint;
        targetPoint.x_ = initialPoint.x_ + vx * t;
        targetPoint.y_ = initialPoint.y_ + vy * t;
        targetPoint.a_ = initialPoint.a_ + va * t;
            
        generate(initialPoint, targetPoint, limits);
    }
    else
    {
        #ifdef PRINT_DEBUG
        PLOGI.printf("Generating const vel trajectory");
        PLOGI.printf("Starting point:");
        initialPoint.print();
        PLOGI.printf("");
        PLOGI.printf("Target velocity: ");
        PLOGI.printf("[vx: ");
        PLOGI.printf(vx);
        PLOGI.printf(", vy: ");
        PLOGI.printf(vy);
        PLOGI.printf(", va: ");
        PLOGI.printf(va);
        PLOGI.printf(", t: ");
        PLOGI.printf(t);
        PLOGI.printf("]");
        PLOGI.printf("");
        #endif

        currentTraj_.xtraj_ = generate_vel_for_time_1D(initialPoint.x_, vx, t, TRAJ_MAX_TRANS_ACC);
        currentTraj_.ytraj_ = generate_vel_for_time_1D(initialPoint.y_, vy, t, TRAJ_MAX_TRANS_ACC);
        currentTraj_.atraj_ = generate_vel_for_time_1D(initialPoint.a_, va, t, TRAJ_MAX_ROT_ACC);

        #ifdef PRINT_DEBUG
        currentTraj_.print();
        #endif

    }
    
}


std::vector<trajParams> TrajectoryGenerator::generate_triangle_1D(float startPos, float endPos, float maxVel, float maxAcc) const
{
    std::vector<trajParams> outTraj;

    float deltaPosition = endPos - startPos;
    int dir = sgn(deltaPosition);
    float halfwayTime = 0.5 * sqrt(2 * fabs(deltaPosition) / maxAcc);

    // First phase - acceleration
    trajParams phase1;
    phase1.t0_ = 0;
    phase1.t_end_ = halfwayTime;
    phase1.p0_ = startPos;
    phase1.v0_ = 0;
    phase1.a_ = dir*maxAcc;
    outTraj.push_back(phase1);

    // Second phase - deceleration
    trajParams phase2;
    phase2.t0_ = halfwayTime;
    phase2.t_end_ = 2*halfwayTime;
    phase2.p0_ = phase1.p0_ + 0.5 * phase1.a_ * halfwayTime * halfwayTime;
    phase2.v0_ = phase1.a_ * halfwayTime;
    phase2.a_ = -1 * phase1.a_;
    outTraj.push_back(phase2);

    return outTraj;
}

std::vector<trajParams> TrajectoryGenerator::generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const
{
    std::vector<trajParams> outTraj;

    float deltaPosition = endPos - startPos;
    int dir = sgn(deltaPosition);
    
    float timeToReachConstVel = maxVel / maxAcc;
    float posToReachConstVel = 0.5 * maxAcc * timeToReachConstVel * timeToReachConstVel;
    float deltaPositionConstVel = fabs(deltaPosition) - 2 * posToReachConstVel;
    float deltaTimeConstVel = fabs(deltaPositionConstVel) / maxVel;

    // First phase - acceleration to max vel
    trajParams phase1;
    phase1.t0_ = 0;
    phase1.t_end_ = timeToReachConstVel;
    phase1.p0_ = startPos;
    phase1.v0_ = 0;
    phase1.a_ = dir*maxAcc;
    outTraj.push_back(phase1);

    // Second phase - constant velocity
    trajParams phase2;
    phase2.t0_ = phase1.t_end_;
    phase2.t_end_ = phase2.t0_ + deltaTimeConstVel;
    phase2.p0_ = phase1.p0_ + dir * posToReachConstVel;
    phase2.v0_ = dir*maxVel;
    phase2.a_ = 0;
    outTraj.push_back(phase2);

    // Third phase - deceleration
    trajParams phase3;
    phase3.t0_ = phase2.t_end_;
    phase3.t_end_ = phase3.t0_ + timeToReachConstVel; // Same time for deceleration as acceleration
    phase3.p0_ = phase2.p0_ + dir * deltaPositionConstVel; // Same position change for deceleration
    phase3.v0_ = phase2.v0_;
    phase3.a_ = -1 * phase1.a_;
    outTraj.push_back(phase3);

    return outTraj;
}

std::vector<trajParams> TrajectoryGenerator::generate_vel_for_time_1D(float startPos, float vel, float time, float maxAcc) const
{
    std::vector<trajParams> outTraj;

    int dir = sgn(vel);
    
    float timeToReachConstVel = fabs(vel) / maxAcc;
    float posToReachConstVel = dir * 0.5 * maxAcc * timeToReachConstVel * timeToReachConstVel;
    float deltaTimeConstVel = time - 2*timeToReachConstVel;
    float deltaPositionConstVel = deltaTimeConstVel * vel;

    // First phase - acceleration to vel
    trajParams phase1;
    phase1.t0_ = 0;
    phase1.t_end_ = timeToReachConstVel;
    phase1.p0_ = startPos;
    phase1.v0_ = 0;
    phase1.a_ = dir*maxAcc;
    outTraj.push_back(phase1);

    // Second phase - constant velocity
    trajParams phase2;
    phase2.t0_ = phase1.t_end_;
    phase2.t_end_ = phase2.t0_ + deltaTimeConstVel;
    phase2.p0_ = phase1.p0_ + posToReachConstVel;
    phase2.v0_ = vel;
    phase2.a_ = 0;
    outTraj.push_back(phase2);

    // Third phase - deceleration
    trajParams phase3;
    phase3.t0_ = phase2.t_end_;
    phase3.t_end_ = phase3.t0_ + timeToReachConstVel; // Same time for deceleration as acceleration
    phase3.p0_ = phase2.p0_ + deltaPositionConstVel;
    phase3.v0_ = phase2.v0_;
    phase3.a_ = -1 * phase1.a_;
    outTraj.push_back(phase3);

    return outTraj;
}

PVTPoint TrajectoryGenerator::lookup(float time)
{
    // Note- this currently doesn't syncronize timescales of different trajectories
    // so they don't all finish at the same time. But that is okay for our purposes right now
    std::vector<float> xvals = lookup_1D(time, currentTraj_.xtraj_);
    std::vector<float> yvals = lookup_1D(time, currentTraj_.ytraj_);
    std::vector<float> avals = lookup_1D(time, currentTraj_.atraj_);

    // Build output structure
    PVTPoint outPoint;
    outPoint.time_ = time;
    outPoint.position_.x_ = xvals[0];
    outPoint.position_.y_ = yvals[0];
    outPoint.position_.a_ = avals[0];
    outPoint.velocity_.x_ = xvals[1];
    outPoint.velocity_.y_ = yvals[1];
    outPoint.velocity_.a_ = avals[1];

    return outPoint;
}

std::vector<float> TrajectoryGenerator::lookup_1D(float time, std::vector<trajParams> traj) const
{
    // Returns [pos, vel] at current time

    int numTrajSegments = traj.size();
    float pos = 0;
    float vel = 0;

    // Check boundary conditions
    // Time is before trajectory starts
    if(time < traj[0].t0_)   
    {
        pos = traj[0].p0_;
        vel = 0;
    }
    // Time is after trajectory ends
    else if(time > traj[numTrajSegments - 1].t_end_)
    {
        trajParams finalTraj = traj[numTrajSegments - 1];
        float dt = finalTraj.t_end_ - finalTraj.t0_;
        pos = finalTraj.p0_ + finalTraj.v0_ * dt + 0.5 * finalTraj.a_ * dt * dt;
        vel = 0;
    }
    else
    {
        // Lookup right traj segment for current time
        for(int i = 0; i < numTrajSegments; i++)
        {
            trajParams curTraj = traj[i];
            if(time >= curTraj.t0_ && time < curTraj.t_end_)
            {
                float dt = time - curTraj.t0_;
                pos = curTraj.p0_ + curTraj.v0_ * dt + 0.5 * curTraj.a_ * dt * dt;
                vel = curTraj.v0_ + curTraj.a_ * dt;
            }
        }
    }

    std::vector<float> rtn = {pos, vel};
    return rtn;
}
