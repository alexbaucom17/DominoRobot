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
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        PVTPoint output = stg.lookup(1.0);
        REQUIRE(output.time == 1.0);

        limits_mode = LIMITS_MODE::FINE;
        ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        output = stg.lookup(1.0);
        REQUIRE(output.time == 1.0);
    }

    SECTION("Final position")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,0};
        Point p2 = {1,2,3};
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time == 60);
        CHECK(output.position.x == Approx(p2.x));
        CHECK(output.position.y == Approx(p2.y));
        CHECK(output.position.a == Approx(p2.a));
        CHECK(output.velocity.vx == Approx(0.0).margin(0.0001));
        CHECK(output.velocity.vy == Approx(0.0).margin(0.0001));
        CHECK(output.velocity.va == Approx(0.0f).margin(0.0001));

        limits_mode = LIMITS_MODE::FINE;
        ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        output = stg.lookup(60);
        REQUIRE(output.time == 60);
        CHECK(output.position.x == Approx(p2.x));
        CHECK(output.position.y == Approx(p2.y));
        CHECK(output.position.a == Approx(p2.a));
        CHECK(output.velocity.vx == Approx(0.0).margin(0.0001));
        CHECK(output.velocity.vy == Approx(0.0).margin(0.0001));
        CHECK(output.velocity.va == Approx(0.0).margin(0.0001));
    }

    SECTION("Zeros")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,0};
        Point p2 = {10,0,0};
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time == 60);
        CHECK(output.position.x == Approx(p2.x));
        CHECK(output.position.y == Approx(p2.y));
        CHECK(output.position.a == Approx(p2.a));
        CHECK(output.velocity.vx == Approx(0));
        CHECK(output.velocity.vy == Approx(0));
        CHECK(output.velocity.va == Approx(0).margin(0.0001));
    }

    SECTION("Lookup too early")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {5,4,3};
        Point p2 = {10,0,0};
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        PVTPoint output = stg.lookup(0);
        REQUIRE(output.time == 0);
        CHECK(output.position.x == Approx(p1.x));
        CHECK(output.position.y == Approx(p1.y));
        CHECK(output.position.a == Approx(p1.a));
        CHECK(output.velocity.vx == Approx(0));
        CHECK(output.velocity.vy == Approx(0));
        CHECK(output.velocity.va == Approx(0).margin(0.0001));

        output = stg.lookup(-1);
        REQUIRE(output.time == -1);
        CHECK(output.position.x == Approx(p1.x));
        CHECK(output.position.y == Approx(p1.y));
        CHECK(output.position.a == Approx(p1.a));
        CHECK(output.velocity.vx == Approx(0));
        CHECK(output.velocity.vy == Approx(0));
        CHECK(output.velocity.va == Approx(0).margin(0.0001));
    }

    SECTION("Angle wrap")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,3};
        Point p2 = {0,0,-3};
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time == 60);
        CHECK(output.position.x == Approx(p2.x));
        CHECK(output.position.y == Approx(p2.y));
        CHECK(output.position.a == Approx(p2.a));
        CHECK(output.velocity.vx == Approx(0));
        CHECK(output.velocity.vy == Approx(0));
        CHECK(output.velocity.va == Approx(0).margin(0.0001));

        for (float t = 0; t < 10; t+=0.1)
        {
            PVTPoint output = stg.lookup(t);
            REQUIRE(fabs(output.position.a) < 3.14);
            REQUIRE(fabs(output.position.a) > 2.99);
        }
    }

    SECTION("Angle non-wrap")
    {
        SmoothTrajectoryGenerator stg;
        Point p1 = {0,0,-1};
        Point p2 = {0,0,1};
        LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;

        bool ok = stg.generatePointToPointTrajectory(p1, p2, limits_mode);
        REQUIRE(ok == true);

        //Way in the future should return the final point
        PVTPoint output = stg.lookup(60);
        REQUIRE(output.time == 60);
        CHECK(output.position.x == Approx(p2.x));
        CHECK(output.position.y == Approx(p2.y));
        CHECK(output.position.a == Approx(p2.a));
        CHECK(output.velocity.vx == Approx(0));
        CHECK(output.velocity.vy == Approx(0));
        CHECK(output.velocity.va == Approx(0).margin(0.0001));

        for (float t = 0; t < 10; t+=0.1)
        {
            PVTPoint output = stg.lookup(t);
            REQUIRE(fabs(output.position.a) < 1.01);
        }
    }
}

TEST_CASE("BuildMotionPlanningProblem", "[trajectory]")
{
    Point p1 = {0,0,0};
    Point p2 = {10,0,-3};
    LIMITS_MODE limits_mode;
    SolverParameters solver;
    solver.num_loops = 10;
    solver.alpha_decay = 0.8;
    solver.beta_decay = 0.8;
    solver.exponent_decay = 0.1;

    Eigen::Vector3f expected_p1 = {0,0,0};
    Eigen::Vector3f expected_p2 = {10,0,-3};

    REQUIRE(static_cast<std::string>(cfg.lookup("name")) == "Test constants");
    REQUIRE(static_cast<float>(cfg.lookup("motion.limit_max_fraction")) == 1.0);

    SECTION ("Coarse mode")
    {
        limits_mode = LIMITS_MODE::COARSE;
        MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, limits_mode, solver);
        REQUIRE(mpp.initialPoint == expected_p1);
        REQUIRE(mpp.targetPoint == expected_p2);
        REQUIRE(mpp.translationalLimits.max_vel == static_cast<float>(cfg.lookup("motion.translation.max_vel.coarse")));
        REQUIRE(mpp.translationalLimits.max_acc == static_cast<float>(cfg.lookup("motion.translation.max_acc.coarse")));
        REQUIRE(mpp.translationalLimits.max_jerk == static_cast<float>(cfg.lookup("motion.translation.max_jerk.coarse")));
        REQUIRE(mpp.rotationalLimits.max_vel ==static_cast<float>( cfg.lookup("motion.rotation.max_vel.coarse")));
        REQUIRE(mpp.rotationalLimits.max_acc == static_cast<float>(cfg.lookup("motion.rotation.max_acc.coarse")));
        REQUIRE(mpp.rotationalLimits.max_jerk == static_cast<float>(cfg.lookup("motion.rotation.max_jerk.coarse")));
        REQUIRE(mpp.solver_params.num_loops == solver.num_loops);
        REQUIRE(mpp.solver_params.alpha_decay == solver.alpha_decay);
        REQUIRE(mpp.solver_params.beta_decay == solver.beta_decay);
        REQUIRE(mpp.solver_params.exponent_decay == solver.exponent_decay);
    }

    SECTION ("Fine mode")
    {
        limits_mode = LIMITS_MODE::FINE;
        MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, limits_mode, solver);
        REQUIRE(mpp.initialPoint == expected_p1);
        REQUIRE(mpp.targetPoint == expected_p2);
        REQUIRE(mpp.translationalLimits.max_vel == static_cast<float>(cfg.lookup("motion.translation.max_vel.fine")));
        REQUIRE(mpp.translationalLimits.max_acc == static_cast<float>(cfg.lookup("motion.translation.max_acc.fine")));
        REQUIRE(mpp.translationalLimits.max_jerk == static_cast<float>(cfg.lookup("motion.translation.max_jerk.fine")));
        REQUIRE(mpp.rotationalLimits.max_vel ==static_cast<float>( cfg.lookup("motion.rotation.max_vel.fine")));
        REQUIRE(mpp.rotationalLimits.max_acc == static_cast<float>(cfg.lookup("motion.rotation.max_acc.fine")));
        REQUIRE(mpp.rotationalLimits.max_jerk == static_cast<float>(cfg.lookup("motion.rotation.max_jerk.fine")));
        REQUIRE(mpp.solver_params.num_loops == solver.num_loops);
        REQUIRE(mpp.solver_params.alpha_decay == solver.alpha_decay);
        REQUIRE(mpp.solver_params.beta_decay == solver.beta_decay);
        REQUIRE(mpp.solver_params.exponent_decay == solver.exponent_decay);
    }
}

TEST_CASE("generateTrajectory", "[trajectory]")
{
    Point p1 = {0,0,0};
    Point p2 = {10,0,-3};
    LIMITS_MODE limits_mode = LIMITS_MODE::COARSE;
    SolverParameters solver = {25, 0.8, 0.8, 0.1};
    MotionPlanningProblem mpp = buildMotionPlanningProblem(p1, p2, limits_mode, solver);

    Trajectory traj = generateTrajectory(mpp);

    REQUIRE(traj.complete == true);
    CHECK(traj.initialPoint == p1);
    CHECK(traj.trans_direction(0) == 1);
    CHECK(traj.trans_direction(1) == 0);
    CHECK(traj.rot_direction == -1);
}

TEST_CASE("generateSCurve", "[trajectory]")
{
    float dist = 10;
    DynamicLimits limits = {1, 2, 8};
    SolverParameters solver = {25, 0.8, 0.8, 0.1};
    SCurveParameters params;
    bool ok = generateSCurve(dist, limits, solver, &params);
    REQUIRE(ok == true);
    REQUIRE(params.v_lim == 1.0);
    REQUIRE(params.a_lim == 2.0);
    REQUIRE(params.j_lim == 8.0);
    float dt_v = 9.25; // Expected values
    float dt_a = 0.25;
    float dt_j = 0.25;
    CHECK(params.switch_points[0].t == 0);
    CHECK(params.switch_points[1].t == dt_j);
    CHECK(params.switch_points[2].t == dt_j + dt_a);
    CHECK(params.switch_points[3].t == 2*dt_j + dt_a);
    CHECK(params.switch_points[4].t == 2*dt_j + dt_a + dt_v);
    CHECK(params.switch_points[5].t == 3*dt_j + dt_a + dt_v);
    CHECK(params.switch_points[6].t == 3*dt_j + 2*dt_a + dt_v);
    CHECK(params.switch_points[7].t == 4*dt_j + 2*dt_a + dt_v);

    SECTION("Zero")
    {
        float dist = 0;
        DynamicLimits limits = {1, 2, 8};
        SolverParameters solver = {25, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim == 0);
        REQUIRE(params.a_lim == 0);
        REQUIRE(params.j_lim == 0);
        for (int i = 0; i < 8; i++)
        {
            CHECK(params.switch_points[i].t == 0);
            CHECK(params.switch_points[i].p == 0);
            CHECK(params.switch_points[i].v == 0);
            CHECK(params.switch_points[i].a == 0);
        }
    }

    SECTION("Very close to zero")
    {
        float dist = 0.000001;
        DynamicLimits limits = {1, 2, 8};
        SolverParameters solver = {25, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim == 0);
        REQUIRE(params.a_lim == 0);
        REQUIRE(params.j_lim == 0);
        for (int i = 0; i < 8; i++)
        {
            CHECK(params.switch_points[i].t == 0);
            CHECK(params.switch_points[i].p == 0);
            CHECK(params.switch_points[i].v == 0);
            CHECK(params.switch_points[i].a == 0);
        }
    }

    SECTION("Small dist that is above min_dist_limit")
    {
        float dist = 0.0001;
        DynamicLimits limits = {4, 2, 1};
        SolverParameters solver = {30, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim < 4);
        REQUIRE(params.a_lim < 2);
        REQUIRE(params.j_lim == 1);
    }

    SECTION("Modified v limit")
    {
        float dist = 10;
        DynamicLimits limits = {3, 1, 1};
        SolverParameters solver = {25, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim < 3);
        REQUIRE(params.a_lim == 1);
        REQUIRE(params.j_lim == 1);
    }

    SECTION("Modified a limit")
    {
        float dist = 10;
        DynamicLimits limits = {1, 2, 1};
        SolverParameters solver = {25, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim == 1);
        REQUIRE(params.a_lim < 2);
        REQUIRE(params.j_lim == 1);
    }

    SECTION("Modified both limits")
    {
        float dist = 10;
        DynamicLimits limits = {4, 2, 1};
        SolverParameters solver = {25, 0.8, 0.8, 0.1};
        SCurveParameters params;
        bool ok = generateSCurve(dist, limits, solver, &params);
        REQUIRE(ok == true);
        REQUIRE(params.v_lim < 4);
        REQUIRE(params.a_lim < 2);
        REQUIRE(params.j_lim == 1);
    }
}

TEST_CASE("populateSwitchTimeParameters", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim = 1.0;
    params.a_lim = 2.0;
    params.j_lim = 8.0;
    float dt_v = 9.25;
    float dt_a = 0.25;
    float dt_j = 0.25;

    populateSwitchTimeParameters(&params, dt_j, dt_a, dt_v);

    REQUIRE(params.switch_points[0].t == Approx(0));
    REQUIRE(params.switch_points[0].p == Approx(0));
    REQUIRE(params.switch_points[0].v == Approx(0));
    REQUIRE(params.switch_points[0].a == Approx(0));

    CHECK(params.switch_points[1].t == Approx(0.25).margin(0.001));
    CHECK(params.switch_points[1].p == Approx(0.02083).margin(0.001));
    CHECK(params.switch_points[1].v == Approx(0.25).margin(0.001));
    CHECK(params.switch_points[1].a == Approx(2.0).margin(0.001));

    CHECK(params.switch_points[2].t == Approx(0.5).margin(0.001));
    CHECK(params.switch_points[2].p == Approx(0.1458).margin(0.001));
    CHECK(params.switch_points[2].v == Approx(0.75).margin(0.001));
    CHECK(params.switch_points[2].a == Approx(2.0).margin(0.001));

    CHECK(params.switch_points[3].t == Approx(0.75).margin(0.001));
    CHECK(params.switch_points[3].p == Approx(0.375).margin(0.001));
    CHECK(params.switch_points[3].v == Approx(1.0).margin(0.001));
    CHECK(params.switch_points[3].a == Approx(0).margin(0.001));

    CHECK(params.switch_points[4].t == Approx(10.0).margin(0.001));
    CHECK(params.switch_points[4].p == Approx(9.625).margin(0.001));
    CHECK(params.switch_points[4].v == Approx(1.0).margin(0.001));
    CHECK(params.switch_points[4].a == Approx(0).margin(0.001));

    CHECK(params.switch_points[5].t == Approx(10.25).margin(0.001));
    CHECK(params.switch_points[5].p == Approx(9.854).margin(0.001));
    CHECK(params.switch_points[5].v == Approx(0.75).margin(0.001));
    CHECK(params.switch_points[5].a == Approx(-2.0).margin(0.001));

    CHECK(params.switch_points[6].t == Approx(10.5).margin(0.001));
    CHECK(params.switch_points[6].p == Approx(9.98).margin(0.001));
    CHECK(params.switch_points[6].v == Approx(0.25).margin(0.001));
    CHECK(params.switch_points[6].a == Approx(-2.0).margin(0.001));

    CHECK(params.switch_points[7].t == Approx(10.75).margin(0.001));
    CHECK(params.switch_points[7].p == Approx(10).margin(0.001));
    CHECK(params.switch_points[7].v == Approx(0).margin(0.001));
    CHECK(params.switch_points[7].a == Approx(0).margin(0.001));

}

TEST_CASE("lookup_1D", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim = 1.0;
    params.a_lim = 1.0;
    params.j_lim = 1.0;
    for (int i = 0; i < 8; i ++)
    {
        params.switch_points[i].a = 1.0;
        params.switch_points[i].v = 1.0;
        params.switch_points[i].p = 1.0;
        params.switch_points[i].t = i;
    }
    // Need to manually set a for const vel region
    params.switch_points[3].a = 0;

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
    params.v_lim = 1.0;
    params.a_lim = 1.0;
    params.j_lim = 1.0;
    for (int i = 0; i < 8; i ++)
    {
        params.switch_points[i].a = 1.0;
        params.switch_points[i].v = 1.0;
        params.switch_points[i].p = 1.0;
        params.switch_points[i].t = i;
    }
    // Need to manually set a for const vel region
    params.switch_points[3].a = 0;

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

TEST_CASE("synchronizeParameters", "[trajectory]")
{
    // These values should all be valid together
    std::vector<float> switch_times {0.000, 0.200, 0.500, 0.700, 6.000, 6.200, 6.500, 6.700};
    float dt_v = 5.3;
    float dt_a = 0.3;
    float dt_j = 0.2;
    float dp = 3.0;
    SCurveParameters valid_params;
    valid_params.v_lim = 0.5;
    valid_params.a_lim = 1.0;
    valid_params.j_lim = 5.0;
    populateSwitchTimeParameters(&valid_params, dt_j, dt_a, dt_v);
    for (int i = 0; i < 8; i++)
    {
        REQUIRE(valid_params.switch_points[i].t == switch_times[i]);
    }
    REQUIRE(valid_params.switch_points[7].p == dp);
    
    SCurveParameters params_to_test = valid_params;
    params_to_test.v_lim = 0.0;
    params_to_test.a_lim = 0.0;
    params_to_test.j_lim = 0.0;

    SECTION("solveInverse")
    {
        bool ok = solveInverse(&params_to_test);
        CHECK(ok == true);
        CHECK(params_to_test.v_lim == Approx(valid_params.v_lim));
        CHECK(params_to_test.a_lim == Approx(valid_params.a_lim));
        CHECK(params_to_test.j_lim == Approx(valid_params.j_lim));
    }
    SECTION("slowDownParamsToMatchTime")
    {
        for (int i = 0; i < 8; i++)
        {
            params_to_test.switch_points[i].t /= 2.0;
        }
        bool ok = slowDownParamsToMatchTime(&params_to_test, switch_times[7]);
        CHECK(ok == true);
        CHECK(params_to_test.switch_points[7].t == switch_times[7]);
        CHECK(params_to_test.switch_points[7].p == Approx(dp));
    }
    SECTION("synchronize")
    {
        for (int i = 0; i < 8; i++)
        {
            params_to_test.switch_points[i].t /= 2.0;
        }
        SCurveParameters params_to_test2 = valid_params;
        bool ok = synchronizeParameters(&params_to_test, &params_to_test2);
        CHECK(ok == true);
        CHECK(params_to_test.switch_points[7].t == switch_times[7]);
        CHECK(params_to_test.switch_points[7].p == Approx(dp));
        CHECK(params_to_test2.switch_points[7].t == switch_times[7]);
        CHECK(params_to_test2.switch_points[7].p == Approx(dp));
        for (int i = 0; i < 8; i++)
        {
            CHECK(params_to_test2.switch_points[i].t == switch_times[i]);
        }
    }
}
