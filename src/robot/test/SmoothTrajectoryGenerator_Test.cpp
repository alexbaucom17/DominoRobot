#include <Catch/catch.hpp>

#include "SmoothTrajectoryGenerator.h"
#include "constants.h"

// void overwriteConfig(std::string& key, float value)
// {
//     libconfig::Setting root = cfg.getRoot();
//     if (root.exists(key))
//     {
//         root.remove(key);
//     }
//     root.add(key, value);
// }

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
    SmoothTrajectoryGenerator stg;
    Point p1 = {0,0,0};
    Point p2 = {1,2,3};
    bool fineMode = false;

    bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
    REQUIRE(ok == true);

    PVTPoint output = stg.lookup(1.0);
    REQUIRE(output.time_ == 1.0);

    // TODO: Verify final point
    // TODO: Test zeros
    // TODO: Verify fine vs coarse
    // TODO: Verify some different parameter values
    // TODO: Verify case where algorithm modifies limits

    //Way in the future should return the final point
    // output = stg.lookup(60);
    // REQUIRE(output.time_ == 60);
    // CHECK(output.position_ == p2);
    // REQUIRE(output.velocity_ == Velocity(0,0,0));
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
}
TEST_CASE("generateSCurve", "[trajectory]")
{
}
TEST_CASE("populateSwitchTimeParameters", "[trajectory]")
{
    SCurveParameters params;
    params.v_lim_ = 1.0;
    params.a_lim_ = 2.0;
    params.j_lim_ = 8.0;
    float dt_v = 10.41 - 0.75;
    float dt_a = 0.25;
    float dt_j = 0.25;

    populateSwitchTimeParameters(&params, dt_j, dt_a, dt_v);

    REQUIRE(params.switch_points_[0].t_ == Approx(0));
    REQUIRE(params.switch_points_[0].p_ == Approx(0));
    REQUIRE(params.switch_points_[0].v_ == Approx(0));
    REQUIRE(params.switch_points_[0].a_ == Approx(0));

    REQUIRE(params.switch_points_[1].t_ == Approx(0.25));
    REQUIRE(params.switch_points_[1].p_ == Approx(0.0208));
    REQUIRE(params.switch_points_[1].v_ == Approx(0.271));
    REQUIRE(params.switch_points_[1].a_ == Approx(2.0));

    REQUIRE(params.switch_points_[2].t_ == Approx(0.5));
    REQUIRE(params.switch_points_[2].p_ == Approx(0.157));
    REQUIRE(params.switch_points_[2].v_ == Approx(0.768));
    REQUIRE(params.switch_points_[2].a_ == Approx(2.0));

    REQUIRE(params.switch_points_[3].t_ == Approx(0.75));
    REQUIRE(params.switch_points_[3].p_ == Approx(0.396));
    REQUIRE(params.switch_points_[3].v_ == Approx(1.0));
    REQUIRE(params.switch_points_[3].a_ == Approx(0));

    REQUIRE(params.switch_points_[4].t_ == Approx(10.41));
    REQUIRE(params.switch_points_[4].p_ == Approx(10.05));
    REQUIRE(params.switch_points_[4].v_ == Approx(1.0));
    REQUIRE(params.switch_points_[4].a_ == Approx(0));

    REQUIRE(params.switch_points_[5].t_ == Approx(10.66));
    REQUIRE(params.switch_points_[5].p_ == Approx(10.28));
    REQUIRE(params.switch_points_[5].v_ == Approx(0.66));
    REQUIRE(params.switch_points_[5].a_ == Approx(-2.0));

    REQUIRE(params.switch_points_[6].t_ == Approx(10.91));
    REQUIRE(params.switch_points_[6].p_ == Approx(10.42));
    REQUIRE(params.switch_points_[6].v_ == Approx(0.29));
    REQUIRE(params.switch_points_[6].a_ == Approx(-2.0));

    REQUIRE(params.switch_points_[7].t_ == Approx(11.16));
    REQUIRE(params.switch_points_[7].p_ == Approx(10.50));
    REQUIRE(params.switch_points_[7].v_ == Approx(0));
    REQUIRE(params.switch_points_[7].a_ == Approx(0));

}
TEST_CASE("synchronizeParameters", "[trajectory]")
{
}
TEST_CASE("mapParameters", "[trajectory]")
{
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
