#ifndef TrajectoryGenerator_h
#define TrajectoryGenerator_h

#include "globals.h"
#include <ArduinoSTL.h>

struct Point
{
    float x_;
    float y_;
    float a_;

    Point(float x=0, float y=0, float a=0)
        : x_(x), y_(y), a_(a)
    {}

    Point& operator=(const Point& p)
    {
        x_ = p.x_;
        y_ = p.y_;
        a_ = p.a_;
        return *this;
    }

    Point& operator+(const Point& p)
    {
        x_ += p.x_;
        y_ += p.y_;
        a_ += p.a_;
        return *this;
    }

    Point& operator-(const Point& p)
    {
        x_ -= p.x_;
        y_ -= p.y_;
        a_ -= p.a_;
        return *this;
    }
};


struct PVTPoint
{
    Point position_;
    Point velocity_;
    float time_;
};

// Parameters for a 1D constant acceleration trajectory
struct trajParams
{
    float p0_;    // Start position
    float v0_;    // Start velocity
    float t0_;    // Start time
    float a_;     // Const acceleration
    float t_end_; // End time
};


class TrajectoryGenerator
{

    public:

        TrajectoryGenerator();
        void generate(const Point& initialPoint, const Point& targetPoint);
        PVTPoint lookup(float time);

    private:

        // Doesn't handle magnitude of 2D trajectories right now, but that
        // can be a future improvement if needed
        struct MultiTrajectory
        {
            std::vector<trajParams> xtraj_;
            std::vector<trajParams> ytraj_;
            std::vector<trajParams> atraj_;
        };
        
        // Helper functions
        std::vector<trajParams> generate_triangle_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<trajParams> generate_trapazoid_1D(float startPos, float endPos, float maxVel, float maxAcc) const;
        std::vector<float> lookup_1D(float time, std::vector<trajParams> traj) const;

        MultiTrajectory currentTraj_;
        const float timeForConstVelTrans_;
        const float posForConstVelTrans_;
        const float timeForConstVelRot_;
        const float posForConstVelRot_;


};

#endif