#include "TrajectoryGenerator.h"
#include <math.h>

TrajectoryGenerator::TrajectoryGenerator()
  : currentTraj_(0),
    timeForConstVelTrans_(MAX_TRANS_SPEED / MAX_TRANS_ACC),
    posForConstVelTrans_(0.5 * MAX_TRANS_ACC * timeForConstVelTrans_ * timeForConstVelTrans_),
    timeForConstVelRot_(MAX_ROT_SPEED / MAX_ROT_ACC),
    posForConstVelRot_(0.5 * MAX_ROT_ACC * timeForConstVelRot_ * timeForConstVelRot_)
{
}

void TrajectoryGenerator::generate(const Point& initialPoint, const Point& targetPoint)
{
    Point deltaPoint = targetPoint - initialPoint;

    // Compute X trajectory
    if(math.abs(deltaPoint.x_) < posForConstVelTrans_)
    {
        currentTraj_.xtraj_ = generate_triangle_1D(initialPoint.x_, targetPoint.x_, MAX_TRANS_SPEED, MAX_TRANS_ACC);
    }
    else
    {
        currentTraj_.xtraj_ = generate_trapazoid_1D(initialPoint.x_, targetPoint.x_, MAX_TRANS_SPEED, MAX_TRANS_ACC);
    }

    // Compute y trajectory
    if(math.abs(deltaPoint.y_) < posForConstVelTrans_)
    {
        currentTraj_.ytraj_ = generate_triangle_1D(initialPoint.y_, targetPoint.y_, MAX_TRANS_SPEED, MAX_TRANS_ACC);
    }
    else
    {
        currentTraj_.ytraj_ = generate_trapazoid_1D(initialPoint.y_, targetPoint.y_, MAX_TRANS_SPEED, MAX_TRANS_ACC);
    }

    // Compute angle trajectory
    if(math.abs(deltaPoint.a_) < posForConstVelRot_)
    {
        currentTraj_.atraj_ = generate_triangle_1D(initialPoint.a_, targetPoint.a_, MAX_ROT_SPEED, MAX_ROT_ACC);
    }
    else
    {
        currentTraj_.atraj_ = generate_trapazoid_1D(initialPoint.a_, targetPoint.a_, MAX_ROT_SPEED, MAX_ROT_ACC);
    }
    
}


std::vector<trajParams> TrajectoryGenerator::generate_triangle_1D(float startPos, float endPos, float maxVel, float maxAcc) const
{
    std::vector<trajParams> outTraj;

    float deltaPosition = endPos - startPos;
    int dir = math.sign(deltaPosition);
    float halfwayTime = math.sqrt(2 * math.abs(deltaPosition) / maxAcc);

    // First phase - acceleration
    trajParams phase1;
    phase1.t0_ = 0;
    phase1.t_end_ = halfwayTime;
    phase1.p0_ = startPos;
    phase1.v0_ = 0;
    phase1.a_ = dir*maxAcc;
    outTraj.extend(phase1);

    // Second phase - deceleration
    trajParams phase2;
    phase2.t0_ = halfwayTime;
    phase2.t_end_ = 2*halfwayTime;
    phase2.p0_ = phase1.p0_ + 0.5 * phase1.a_ * halfwayTime * halfwayTime;
    phase2.v0_ = phase1.a_ * halfwayTime;
    phase2.a_ = -1 * phase1.a_;
    outTraj.extend(phase2);

    return outTraj;
}

std::vector<trajParams> TrajectoryGenerator::generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const
{
    std::vector<trajParams> outTraj;

    float deltaPosition = endPos - startPos;
    int dir = math.sign(deltaPosition);
    
    float timeToReachConstVel = maxVel / maxAcc;
    float posToReachConstVel = 0.5 * maxAcc * timeToReachConstVel * timeToReachConstVel;
    float deltaPositionConstVel = math.abs(deltaPosition) - 2 * posToReachConstVel;
    float deltaTimeConstVel = math.abs(deltaPosition) / maxVel;

    // First phase - acceleration to max vel
    trajParams phase1;
    phase1.t0_ = 0;
    phase1.t_end_ = timeToReachConstVel;
    phase1.p0_ = startPos;
    phase1.v0_ = 0;
    phase1.a_ = dir*maxAcc;
    outTraj.extend(phase1);

    // Second phase - constant velocity
    trajParams phase2;
    phase2.t0_ = phase1.t_end_;
    phase2.t_end_ = phase2.t0_ + deltaTimeConstVel;
    phase2.p0_ = phase1.p0_ + dir * posToReachConstVel;
    phase2.v0_ = maxVel;
    phase2.a_ = 0;
    outTraj.extend(phase2);

    // Third phase - deceleration
    trajParams phase3;
    phase3.t0_ = phase2.t_end_;
    phase3.t_end_ = phase3.t0_ + timeToReachConstVel; // Same time for deceleration as acceleration
    phase3.p0_ = phase2.p0_ + dir * deltaPositionConstVel; // Same position change for deceleration
    phase3.v0_ = maxVel;
    phase3.a_ = -1 * phase1.a_;
    outTraj.extend(phase3);

    return outTraj;
}

PVTPoint TrajectoryGenerator::lookup(float time)
{
    // TODO: Handle different time scales between trajectories
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

std::vector<float> TrajectoryGenerator:lookup_1D(float time, std::vector<trajParams> traj) const
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
    
    return std::vector<float>(pos, vel);
}