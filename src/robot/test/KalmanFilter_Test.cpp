#include <Catch/catch.hpp>
#include <Eigen/Dense>

#include "KalmanFilter.h"


TEST_CASE( "Simple kalman filter tests", "[kalman]" ) 
{

    Eigen::Matrix3f A = Eigen::Matrix3f::Identity(); 
    Eigen::Matrix3f B = Eigen::Matrix3f::Identity(); 
    Eigen::Matrix3f C = Eigen::Matrix3f::Identity(); 
    Eigen::Matrix3f Q = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f P = Eigen::Matrix3f::Identity(); 
    KalmanFilter kf = KalmanFilter(A, B, C, Q, R, P);
    kf.init();

    REQUIRE( kf.state() == Eigen::Vector3f::Zero() );
    REQUIRE( kf.time() == 0.0 );

    SECTION("Predict once") 
    {
        double dt = 0.1;
        Eigen::Vector3f u = {1,0,0};
        kf.predict(dt, B, u);

        REQUIRE( kf.time() == 0.1 );
        Eigen::Vector3f expected = {1,0,0};
        REQUIRE( kf.state() == expected);
    }
}

