#include "KalmanFilter.h"
#include "utils.h"
#include <plog/Log.h>

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::Matrix3f& A,
    const Eigen::Matrix3f& B,
    const Eigen::Matrix3f& C,
    const Eigen::Matrix3f& Q,
    const Eigen::Matrix3f& R,
    const Eigen::Matrix3f& P)
  : A(A), B(B), C(C), Q(Q), R(R), P0(P),
    dt(dt),
    I(Eigen::Matrix3f::Identity())
{
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, Eigen::Vector3f x0) 
{
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
}

void KalmanFilter::init() 
{
  x_hat = Eigen::Vector3f::Zero();
  P = P0;
  t0 = 0;
  t = t0;
}

void KalmanFilter::predict(double dt, const Eigen::Matrix3f& B, const Eigen::Vector3f& u)
{
  this->B = B;
  this->dt = dt;
  x_hat_new = A * x_hat + B * u;
  P = A*P*A.transpose() + Q;
  t += dt;

  // Customize kalman filter to handle 3rd element as an angle and wrap between +/- pi
  x_hat_new(2,0) = wrap_angle(x_hat_new(2,0));
  
  x_hat = x_hat_new;
}

void KalmanFilter::update(const Eigen::Vector3f& y, const Eigen::Matrix3f& R) 
{

#ifdef PRINT_DEBUG
  PLOGI.printf("xhat rows: ");
  PLOGI.printf(x_hat.get_rows());
  PLOGI.printf(" cols: ");
  PLOGI.printf(x_hat.get_cols());
  PLOGI.printf("");

  // Note print for a single column matrix doesn't work for some reason
  PLOGI.printf("State estimate before update");
  PLOGI.printf("[X: ");
  PLOGI.printf(x_hat(0,0), 4);
  PLOGI.printf(", Y: ");
  PLOGI.printf(x_hat(1,0), 4);
  PLOGI.printf(", A: ");
  PLOGI.printf(x_hat(2,0), 4);
  PLOGI.printf("]");
#endif

  this->R = R;
  Eigen::Matrix3f tmp = C*P*C.transpose() + R;
  K = P*C.transpose()*(tmp.inverse());
  x_hat_new += K * (y - C*x_hat);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

#ifdef PRINT_DEBUG
  PLOGI.printf("R matrix");
  PLOGI.printf(R);

  PLOGI.printf("P matrix");
  PLOGI.printf(P);

  PLOGI.printf("K matrix");
  PLOGI.printf(K);

  // PLOGI.printf("xhat rows: ");
  // PLOGI.printf(x_hat.get_rows());
  // PLOGI.printf(" cols: ");
  // PLOGI.printf(x_hat.get_cols());
  // PLOGI.printf("");
  
  PLOGI.printf("State estimate after update");
  PLOGI.printf("[X: ");
  PLOGI.printf(x_hat(0,0));
  PLOGI.printf(", Y: ");
  PLOGI.printf(x_hat(1,0));
  PLOGI.printf(", A: ");
  PLOGI.printf(x_hat(2,0));
  PLOGI.printf("]");
#endif
}
