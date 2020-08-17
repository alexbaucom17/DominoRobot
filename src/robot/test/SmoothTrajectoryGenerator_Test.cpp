#include <Catch/catch.hpp>

#include "SmoothTrajectoryGenerator.h"
#include "constants.h"

// This stuff is needed to correctly print a custom type in Catch for debug
namespace Catch {
template<>
struct StringMaker<Point> 
{
    static std::string convert(Point const& p) 
    {
        return p.toString();
    }
};
template<>
struct StringMaker<Velocity> 
{
    static std::string convert(Velocity const& p) 
    {
        return p.toString();
    }
};
}

TEST_CASE("SmoothTrajectoryGenerator class", "[trajectory]")
{
    SECTION("Smoke test - don't crash")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {3,4,5};
        Point p2 = {1,2,3};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        PVTPoint output = stg.lookup(1.0);
        REQUIRE(output.time_ == 1.0);

        fineMode = true;
        ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        output = stg.lookup(1.0);
        REQUIRE(output.time_ == 1.0);
    }

    SECTION("Final position")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,0};
        Point p2 = {1,2,3};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time_ == 60);
        CHECK(output.position_.x_ == Approx(p2.x_));
        CHECK(output.position_.y_ == Approx(p2.y_));
        CHECK(output.position_.a_ == Approx(p2.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));

        fineMode = true;
        ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        output = stg.lookup(60);
        REQUIRE(output.time_ == 60);
        CHECK(output.position_.x_ == Approx(p2.x_));
        CHECK(output.position_.y_ == Approx(p2.y_));
        CHECK(output.position_.a_ == Approx(p2.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));
    }

    SECTION("Zeros")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,0};
        Point p2 = {10,0,0};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time_ == 60);
        CHECK(output.position_.x_ == Approx(p2.x_));
        CHECK(output.position_.y_ == Approx(p2.y_));
        CHECK(output.position_.a_ == Approx(p2.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));
    }

    SECTION("Lookup too early")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {5,4,3};
        Point p2 = {10,0,0};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        PVTPoint output = stg.lookup(0);
        REQUIRE(output.time_ == 0);
        CHECK(output.position_.x_ == Approx(p1.x_));
        CHECK(output.position_.y_ == Approx(p1.y_));
        CHECK(output.position_.a_ == Approx(p1.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));

        output = stg.lookup(-1);
        REQUIRE(output.time_ == -1);
        CHECK(output.position_.x_ == Approx(p1.x_));
        CHECK(output.position_.y_ == Approx(p1.y_));
        CHECK(output.position_.a_ == Approx(p1.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));
    }

    SECTION("Angle wrap")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,3};
        Point p2 = {0,0,-3};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time_ == 60);
        CHECK(output.position_.x_ == Approx(p2.x_));
        CHECK(output.position_.y_ == Approx(p2.y_));
        CHECK(output.position_.a_ == Approx(p2.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));

        for (float t = 0; t < 10; t+=0.1)
        {
            PVTPoint output = stg.lookup(t);
            REQUIRE(fabs(output.position_.a_) < 3.14);
            REQUIRE(fabs(output.position_.a_) > 2.99);
        }
    }

    SECTION("Angle non-wrap")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,-1};
        Point p2 = {0,0,1};
        bool fineMode = false;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time_ == 60);
        CHECK(output.position_.x_ == Approx(p2.x_));
        CHECK(output.position_.y_ == Approx(p2.y_));
        CHECK(output.position_.a_ == Approx(p2.a_));
        CHECK(output.velocity_.vx_ == Approx(0));
        CHECK(output.velocity_.vy_ == Approx(0));
        CHECK(output.velocity_.va_ == Approx(0).margin(0.0001));

        for (float t = 0; t < 10; t+=0.1)
        {
            PVTPoint output = stg.lookup(t);
            REQUIRE(fabs(output.position_.a_) < 1.01);
        }
    }
}

TEST_CASE("BuildMotionPlanningProblem", "[trajectory]")
{
    Point p1 = {0,0,0};
    Point p2 = {10,0,-3};
    bool fineMode;
    SolverParameters solver;
    solver.num_loops_ = 10;
    solver.alpha_decay_ = 0.8;
    solver.beta_decay_ = 0.8;

    Eigen::Vector3f expected_p1 = {0,0,0};
    Eigen::Vector3f expected_p2 = {10,0,-3};

    REQUIRE(static_cast<std::string>(cfg.lookup("name")) == "Test constants");
    REQUIRE(static_cast<float>(cfg.lookup("motion.limit_max_fraction")) == 1.0);

    SECTION ("Coarse mode")
    {
        fineMode = false;
        MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, fineMode, solver);
        REQUIRE(mpp.initialPoint_ == expected_p1);
        REQUIRE(mpp.targetPoint_ == expected_p2);
        REQUIRE(mpp.translationalLimits_.max_vel_ == static_cast<float>(cfg.lookup("motion.translation.max_vel.coarse")));
        REQUIRE(mpp.translationalLimits_.max_acc_ == static_cast<float>(cfg.lookup("motion.translation.max_acc.coarse")));
        REQUIRE(mpp.translationalLimits_.max_jerk_ == static_cast<float>(cfg.lookup("motion.translation.max_jerk.coarse")));
        REQUIRE(mpp.rotationalLimits_.max_vel_ ==static_cast<float>( cfg.lookup("motion.rotation.max_vel.coarse")));
        REQUIRE(mpp.rotationalLimits_.max_acc_ == static_cast<float>(cfg.lookup("motion.rotation.max_acc.coarse")));
        REQUIRE(mpp.rotationalLimits_.max_jerk_ == static_cast<float>(cfg.lookup("motion.rotation.max_jerk.coarse")));
        REQUIRE(mpp.solver_params_.num_loops_ == solver.num_loops_);
        REQUIRE(mpp.solver_params_.alpha_decay_ == solver.alpha_decay_);
        REQUIRE(mpp.solver_params_.beta_decay_ == solver.beta_decay_);
    }

    SECTION ("Fine mode")
    {
        fineMode = true;
        MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, fineMode, solver);
        REQUIRE(mpp.initialPoint_ == expected_p1);
        REQUIRE(mpp.targetPoint_ == expected_p2);
        REQUIRE(mpp.translationalLimits_.max_vel_ == static_cast<float>(cfg.lookup("motion.translation.max_vel.fine")));
        REQUIRE(mpp.translationalLimits_.max_acc_ == static_cast<float>(cfg.lookup("motion.translation.max_acc.fine")));
        REQUIRE(mpp.translationalLimits_.max_jerk_ == static_cast<float>(cfg.lookup("motion.translation.max_jerk.fine")));
        REQUIRE(mpp.rotationalLimits_.max_vel_ ==static_cast<float>( cfg.lookup("motion.rotation.max_vel.fine")));
        REQUIRE(mpp.rotationalLimits_.max_acc_ == static_cast<float>(cfg.lookup("motion.rotation.max_acc.fine")));
        REQUIRE(mpp.rotationalLimits_.max_jerk_ == static_cast<float>(cfg.lookup("motion.rotation.max_jerk.fine")));
        REQUIRE(mpp.solver_params_.num_loops_ == solver.num_loops_);
        REQUIRE(mpp.solver_params_.alpha_decay_ == solver.alpha_decay_);
        REQUIRE(mpp.solver_params_.beta_decay_ == solver.beta_decay_);
    }
}

TEST_CASE("generateTrajectory", "[trajectory]")
{
    Point p1 = {0,0,0};
    Point p2 = {10,0,-3};
    bool fineMode = false;
    SolverParameters solver = {10, 0.8, 0.8};
    MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, fineMode, solver);

    Trajectory traj = generateTrajectory(mpp);

    REQUIRE(traj.complete_ == true);
    CHECK(traj.initialPoint_ == p1);
    CHECK(traj.trans_direction_(0) == 1);
    CHECK(traj.trans_direction_(1) == 0);
    CHECK(traj.rot_direction_ == -1);
}

TEST_CASE("generateSCurve", "[trajectory]")
{
    float dist = 10;
    DynamicLimits limits = {1, 2, 8};
    SolverParameters solver = {10, 0.8, 0.8};
    SCurveParameters params;
    bool ok = generateSCurve(dist, limits, solver, &params);
    REQUIRE(ok == true);
    REQUIRE(params.v_lim_ == 1.0);
    REQUIRE(params.a_lim_ == 2.0);
    REQUIRE(params.j_lim_ == 8.0);
    float dt_v = 9.25; // Expected values
    float dt_a = 0.25;
    float dt_j = 0.25;
    CHECK(params.switch_points_[0].t_ == 0);
    CHECK(params.switch_points_[1].t_ == dt_j);
    CHECK(params.switch_points_[2].t_ == dt_j + dt_a);
    CHECK(params.switch_points_[3].t_ == 2*dt_j + dt_a);
    CHECK(params.switch_points_[4].t_ == 2*dt_j + dt_a + dt_v);
    CHECK(params.switch_points_[5].t_ == 3*dt_j + dt_a + dt_v);
    CHECK(params.switch_points_[6].t_ == 3*dt_j + 2*dt_a + dt_v);
    CHECK(params.switch_points_[7].t_ == 4*dt_j + 2*dt_a + dt_v);

    SECTION("Zero")
    {
        float dist = 0;
        DynamicLimits limits = {1, 2, 8};
        SolverParameters solver = {10, 0.8, 0.8};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim_ == 0);
        REQUIRE(params.a_lim_ == 0);
        REQUIRE(params.j_lim_ == 0);
        for (int i = 0; i < 8; i++)
        {
            CHECK(params.switch_points_[i].t_ == 0);
            CHECK(params.switch_points_[i].p_ == 0);
            CHECK(params.switch_points_[i].v_ == 0);
            CHECK(params.switch_points_[i].a_ == 0);
        }
    }

    SECTION("Modified v limit")
    {
        float dist = 10;
        DynamicLimits limits = {3, 1, 1};
        SolverParameters solver = {10, 0.8, 0.8};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim_ < 3);
        REQUIRE(params.a_lim_ == 1);
        REQUIRE(params.j_lim_ == 1);
    }

    SECTION("Modified a limit")
    {
        float dist = 10;
        DynamicLimits limits = {1, 2, 1};
        SolverParameters solver = {10, 0.8, 0.8};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim_ == 1);
        REQUIRE(params.a_lim_ < 2);
        REQUIRE(params.j_lim_ == 1);
    }

    SECTION("Modified both limits")
    {
        float dist = 10;
        DynamicLimits limits = {4, 2, 1};
        SolverParameters solver = {10, 0.8, 0.8};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim_ < 4);
        REQUIRE(params.a_lim_ < 2);
        REQUIRE(params.j_lim_ == 1);
    }
}

TEST_CASE("populateSwitchTimeParameters", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim_ = 1.0;
    params.a_lim_ = 2.0;
    params.j_lim_ = 8.0;
    float dt_v = 9.25;
    float dt_a = 0.25;
    float dt_j = 0.25;

    populateSwitchTimeParameters(&params, dt_j, dt_a, dt_v);

    REQUIRE(params.switch_points_[0].t_ == Approx(0));
    REQUIRE(params.switch_points_[0].p_ == Approx(0));
    REQUIRE(params.switch_points_[0].v_ == Approx(0));
    REQUIRE(params.switch_points_[0].a_ == Approx(0));

    CHECK(params.switch_points_[1].t_ == Approx(0.25).margin(0.001));
    CHECK(params.switch_points_[1].p_ == Approx(0.02083).margin(0.001));
    CHECK(params.switch_points_[1].v_ == Approx(0.25).margin(0.001));
    CHECK(params.switch_points_[1].a_ == Approx(2.0).margin(0.001));

    CHECK(params.switch_points_[2].t_ == Approx(0.5).margin(0.001));
    CHECK(params.switch_points_[2].p_ == Approx(0.1458).margin(0.001));
    CHECK(params.switch_points_[2].v_ == Approx(0.75).margin(0.001));
    CHECK(params.switch_points_[2].a_ == Approx(2.0).margin(0.001));

    CHECK(params.switch_points_[3].t_ == Approx(0.75).margin(0.001));
    CHECK(params.switch_points_[3].p_ == Approx(0.375).margin(0.001));
    CHECK(params.switch_points_[3].v_ == Approx(1.0).margin(0.001));
    CHECK(params.switch_points_[3].a_ == Approx(0).margin(0.001));

    CHECK(params.switch_points_[4].t_ == Approx(10.0).margin(0.001));
    CHECK(params.switch_points_[4].p_ == Approx(9.625).margin(0.001));
    CHECK(params.switch_points_[4].v_ == Approx(1.0).margin(0.001));
    CHECK(params.switch_points_[4].a_ == Approx(0).margin(0.001));

    CHECK(params.switch_points_[5].t_ == Approx(10.25).margin(0.001));
    CHECK(params.switch_points_[5].p_ == Approx(9.854).margin(0.001));
    CHECK(params.switch_points_[5].v_ == Approx(0.75).margin(0.001));
    CHECK(params.switch_points_[5].a_ == Approx(-2.0).margin(0.001));

    CHECK(params.switch_points_[6].t_ == Approx(10.5).margin(0.001));
    CHECK(params.switch_points_[6].p_ == Approx(9.98).margin(0.001));
    CHECK(params.switch_points_[6].v_ == Approx(0.25).margin(0.001));
    CHECK(params.switch_points_[6].a_ == Approx(-2.0).margin(0.001));

    CHECK(params.switch_points_[7].t_ == Approx(10.75).margin(0.001));
    CHECK(params.switch_points_[7].p_ == Approx(10).margin(0.001));
    CHECK(params.switch_points_[7].v_ == Approx(0).margin(0.001));
    CHECK(params.switch_points_[7].a_ == Approx(0).margin(0.001));

}

TEST_CASE("lookup_1D", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim_ = 1.0;
    params.a_lim_ = 1.0;
    params.j_lim_ = 1.0;
    for (int i = 0; i < 8; i ++)
    {
        params.switch_points_[i].a_ = 1.0;
        params.switch_points_[i].v_ = 1.0;
        params.switch_points_[i].p_ = 1.0;
        params.switch_points_[i].t_ = i;
    }
    // Need to manually set a_ for const vel region
    params.switch_points_[3].a_ = 0;

    SECTION("Region 1 - Const Positive Jerk")
    {
        int region = 1;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.625;
        float expected_p = 1.645;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 2 - Const Positive Acc")
    {
        int region = 2;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.5;
        float expected_p = 1.625;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 3 - Const Negative Jerk")
    {
        int region = 3;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.375;
        float expected_p = 1.604;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 4 - Const Vel")
    {
        int region = 4;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.0;
        float expected_p = 1.5;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 5 - Const Negative Jerk")
    {
        int region = 5;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.375;
        float expected_p = 1.604;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 6 - Const Negative Acc")
    {
        int region = 6;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.5;
        float expected_p = 1.625;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 7 - Const Positive Jerk")
    {
        int region = 7;
        float time = static_cast<float>(region-1) + 0.5;
        std::vector<float> test_vec = lookup_1D(time, params);
        float expected_v = 1.625;
        float expected_p = 1.645;
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[0] == Approx(expected_p).margin(0.001));
    }
}

TEST_CASE("computeKinematicsBasedOnRegion", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim_ = 1.0;
    params.a_lim_ = 1.0;
    params.j_lim_ = 1.0;
    for (int i = 0; i < 8; i ++)
    {
        params.switch_points_[i].a_ = 1.0;
        params.switch_points_[i].v_ = 1.0;
        params.switch_points_[i].p_ = 1.0;
        params.switch_points_[i].t_ = i;
    }
    // Need to manually set a_ for const vel region
    params.switch_points_[3].a_ = 0;

    SECTION("Region 1 - Const Positive Jerk")
    {
        int region = 1;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 1.5;
        float expected_v = 1.625;
        float expected_p = 1.645;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 2 - Const Positive Acc")
    {
        int region = 2;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 1.0;
        float expected_v = 1.5;
        float expected_p = 1.625;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 3 - Const Negative Jerk")
    {
        int region = 3;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 0.5;
        float expected_v = 1.375;
        float expected_p = 1.604;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 4 - Const Vel")
    {
        int region = 4;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 0;
        float expected_v = 1.0;
        float expected_p = 1.5;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 5 - Const Negative Jerk")
    {
        int region = 5;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 0.5;
        float expected_v = 1.375;
        float expected_p = 1.604;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 6 - Const Negative Acc")
    {
        int region = 6;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = -1.0;
        float expected_v = 1.5;
        float expected_p = 1.625;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
    SECTION("Region 7 - Const Positive Jerk")
    {
        int region = 7;
        float dt = 0.5;
        std::vector<float> test_vec = computeKinematicsBasedOnRegion(params, region, dt);
        float expected_a = 1.5;
        float expected_v = 1.625;
        float expected_p = 1.645;
        REQUIRE(test_vec[0] == Approx(expected_a).margin(0.001));
        REQUIRE(test_vec[1] == Approx(expected_v).margin(0.001));
        REQUIRE(test_vec[2] == Approx(expected_p).margin(0.001));
    }
}


// TEST_CASE("synchronizeParameters", "[trajectory]")
// {
//     SECTION("Monotonic - First arg")
//     {
//         // Generate one set of parameters
//         float dist = 10;
//         DynamicLimits limits = {1, 2, 8};
//         SolverParameters solver = {10, 0.8, 0.8};
//         SCurveParameters params1;
//         bool ok = generateSCurve(dist, limits, solver, &params1);
//         REQUIRE(ok == true);
//         CHECK(params1.v_lim_ == limits.max_vel_);
//         CHECK(params1.a_lim_ == limits.max_acc_);
//         CHECK(params1.j_lim_ == limits.max_jerk_);

//         // Make new set of params - make sure all dt values are smaller than the first one so 
//         // it maps params1 times to param2 
//         SCurveParameters params2;
//         params2.switch_points_[7].p_ = dist;
//         float dt_v = 9;
//         float dt_a = 0.05;
//         float dt_j = 0.05;
//         params2.switch_points_[0].t_ = 0;
//         params2.switch_points_[1].t_ = dt_j;
//         params2.switch_points_[2].t_ = dt_j + dt_a;
//         params2.switch_points_[3].t_ = 2*dt_j + dt_a;
//         params2.switch_points_[4].t_ = 2*dt_j + dt_a + dt_v;
//         params2.switch_points_[5].t_ = 3*dt_j + dt_a + dt_v;
//         params2.switch_points_[6].t_ = 3*dt_j + 2*dt_a + dt_v;
//         params2.switch_points_[7].t_ = 4*dt_j + 2*dt_a + dt_v;

//         // Synchronize parameters
//         ok = synchronizeParameters(&params1, &params2);
//         REQUIRE(ok == true);

//         // Verify limits match expectation - i.e. params2 got mapped to slower values
//         CHECK(params2.v_lim_ == Approx(limits.max_vel_).epsilon(0.01));
//         CHECK(params2.a_lim_ == Approx(limits.max_acc_).epsilon(0.01));
//         CHECK(params2.j_lim_ == Approx(limits.max_jerk_).epsilon(0.01));

//     }
//     SECTION("Monotonic - Second arg")
//     {
//         // Generate one set of parameters
//         float dist = 10;
//         DynamicLimits limits = {1, 2, 8};
//         SolverParameters solver = {10, 0.8, 0.8};
//         SCurveParameters params1;
//         bool ok = generateSCurve(dist, limits, solver, &params1);
//         REQUIRE(ok == true);
//         CHECK(params1.v_lim_ == limits.max_vel_);
//         CHECK(params1.a_lim_ == limits.max_acc_);
//         CHECK(params1.j_lim_ == limits.max_jerk_);

//         // Make new set of params - make sure all dt values are smaller than the first one so 
//         // it maps params1 times to param2 
//         SCurveParameters params2;
//         params2.switch_points_[7].p_ = dist;
//         float dt_v = 9;
//         float dt_a = 0.05;
//         float dt_j = 0.05;
//         params2.switch_points_[0].t_ = 0;
//         params2.switch_points_[1].t_ = dt_j;
//         params2.switch_points_[2].t_ = dt_j + dt_a;
//         params2.switch_points_[3].t_ = 2*dt_j + dt_a;
//         params2.switch_points_[4].t_ = 2*dt_j + dt_a + dt_v;
//         params2.switch_points_[5].t_ = 3*dt_j + dt_a + dt_v;
//         params2.switch_points_[6].t_ = 3*dt_j + 2*dt_a + dt_v;
//         params2.switch_points_[7].t_ = 4*dt_j + 2*dt_a + dt_v;

//         // Synchronize parameters
//         ok = synchronizeParameters(&params1, &params2);
//         REQUIRE(ok == true);

//         // Verify limits match expectation - i.e. params2 got mapped to slower values
//         CHECK(params2.v_lim_ == Approx(limits.max_vel_).epsilon(0.01));
//         CHECK(params2.a_lim_ == Approx(limits.max_acc_).epsilon(0.01));
//         CHECK(params2.j_lim_ == Approx(limits.max_jerk_).epsilon(0.01));

//     }
//     SECTION ("Nonmonotonic")
//     {

//     }
// }

// TEST_CASE("mapParameters", "[trajectory]")
// {
//     // Generate one set of parameters
//     float dist = 10;
//     DynamicLimits limits = {1, 2, 8};
//     SolverParameters solver = {10, 0.8, 0.8};
//     SCurveParameters params1;
//     bool ok = generateSCurve(dist, limits, solver, &params1);
//     REQUIRE(ok == true);
//     CHECK(params1.v_lim_ == limits.max_vel_);
//     CHECK(params1.a_lim_ == limits.max_acc_);
//     CHECK(params1.j_lim_ == limits.max_jerk_);

//     // Solve the inverse
//     ok = solveInverse(&params1);
//     REQUIRE(ok == true);

//     // Verify times match expectation
//     float dt_v = 9.25;
//     float dt_a = 0.25;
//     float dt_j = 0.25;
//     CHECK(params1.switch_points_[0].t_ == 0);
//     CHECK(params1.switch_points_[1].t_ == dt_j);
//     CHECK(params1.switch_points_[2].t_ == dt_j + dt_a);
//     CHECK(params1.switch_points_[3].t_ == 2*dt_j + dt_a);
//     CHECK(params1.switch_points_[4].t_ == 2*dt_j + dt_a + dt_v);
//     CHECK(params1.switch_points_[5].t_ == 3*dt_j + dt_a + dt_v);
//     CHECK(params1.switch_points_[6].t_ == 3*dt_j + 2*dt_a + dt_v);
//     CHECK(params1.switch_points_[7].t_ == 4*dt_j + 2*dt_a + dt_v);

//     // Verify limits match expectation
//     CHECK(params1.v_lim_ == Approx(limits.max_vel_).epsilon(0.01));
//     CHECK(params1.a_lim_ == Approx(limits.max_acc_).epsilon(0.01));
//     CHECK(params1.j_lim_ == Approx(limits.max_jerk_).epsilon(0.01));
// }
