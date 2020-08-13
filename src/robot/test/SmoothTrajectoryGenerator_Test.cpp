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

TEST_CASE("Simple case - don't crash", "[trajectory]")
{
    SmoothTrajectoryGenerator stg;
    Point p1 = {0,0,0};
    Point p2 = {1,2,3};
    bool fineMode = false;

    bool ok = stg.generatePointToPointTrajectory(p1, p2, fineMode);
    REQUIRE(ok == true);

    PVTPoint output = stg.lookup(1.0);
    REQUIRE(output.time_ == 1.0);

    // Way in the future should return the final point
    output = stg.lookup(60);
    REQUIRE(output.time_ == 60);
    CHECK(output.position_ == p2);
    REQUIRE(output.velocity_ == Velocity(0,0,0));
}