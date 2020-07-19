#include "KalmanFilter.h"
#include "utils.h"
#include <spdlog/spdlog.h>

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
    I(Eigen::Matrix3f::Identity
{
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, mat x0) 
{
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
}

void KalmanFilter::init() 
{
  x_hat = Eigen::Vector3f::zero();
  P = P0;
  t0 = 0;
  t = t0;
}

void KalmanFilter::predict(double dt, const Eigen::Matrix3f& B, const Eigen::Vector3f& u)
{
  this->B = B;
  this->dt = dt;
  x_hat_new = A * x_hat + B * u;
  P = A*P*A.t() + Q;
  t += dt;

  // Customize kalman filter to handle 3rd element as an angle and wrap between +/- pi
  x_hat_new(2,0) = wrap_angle(x_hat_new(2,0));
  
  x_hat = x_hat_new;
}

void KalmanFilter::update(const Eigen::Vector3f& y, const Eigen::Matrix3f& R) 
{

#ifdef PRINT_DEBUG
  spdlog::logger* logger = spdlog::get("robot_logger")
  logger->info("xhat rows: ");
  logger->info(x_hat.get_rows());
  logger->info(" cols: ");
  logger->info(x_hat.get_cols());
  logger->info("");

  // Note print for a single column matrix doesn't work for some reason
  logger->info("State estimate before update");
  logger->info("[X: ");
  logger->info(x_hat(0,0), 4);
  logger->info(", Y: ");
  logger->info(x_hat(1,0), 4);
  logger->info(", A: ");
  logger->info(x_hat(2,0), 4);
  logger->info("]");
#endif

  this->R = R;
  K = P*C.t()*(C*P*C.t() + R).inv();
  x_hat_new += K * (y - C*x_hat);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

#ifdef PRINT_DEBUG
  logger->info("R matrix");
  logger->info(R);

  logger->info("P matrix");
  logger->info(P);

  logger->info("K matrix");
  logger->info(K);

  // logger->info("xhat rows: ");
  // logger->info(x_hat.get_rows());
  // logger->info(" cols: ");
  // logger->info(x_hat.get_cols());
  // logger->info("");
  
  logger->info("State estimate after update");
  logger->info("[X: ");
  logger->info(x_hat(0,0));
  logger->info(", Y: ");
  logger->info(x_hat(1,0));
  logger->info(", A: ");
  logger->info(x_hat(2,0));
  logger->info("]");
#endif
}
