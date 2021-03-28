#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXf& A,
    const Eigen::MatrixXf& B,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXf& Q,
    const Eigen::MatrixXf& R )
:  KalmanFilter(A.rows(), C.rows())
{
    A_ = A; 
    B_ = B; 
    C_ = C; 
    Q_ = Q; 
    R_ = R; 
}

KalmanFilter::KalmanFilter(int n, int m) 
: n_(n),
  m_(m),
  A_(Eigen::MatrixXf::Identity(n,n)), 
  B_(Eigen::MatrixXf::Zero(n,n)), 
  C_(Eigen::MatrixXf::Zero(m,n)), 
  I_(Eigen::MatrixXf::Identity(n,n)),
  K_(Eigen::MatrixXf::Zero(n,m)),
  P_(Eigen::MatrixXf::Identity(n,n)),
  Q_(Eigen::MatrixXf::Identity(n,n)), 
  R_(Eigen::MatrixXf::Identity(m,m)), 
  S_(Eigen::MatrixXf::Identity(m,m)),
  x_hat_(Eigen::VectorXf::Zero(n))
{}

void KalmanFilter::predict(const Eigen::VectorXf& u)
{
    x_hat_ = A_ * x_hat_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXf& y) 
{
    S_ = C_ * P_ * C_.transpose() + R_;
    K_ = P_ * C_.transpose() * S_.inverse();
    x_hat_ = x_hat_ + K_ * (y - C_ * x_hat_);
    P_ = (I_ - K_ * C_) * P_;
}