#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R )
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
  A_(Eigen::MatrixXd::Identity(n,n)), 
  B_(Eigen::MatrixXd::Zero(n,1)), 
  C_(Eigen::MatrixXd::Zero(m,n)), 
  I_(Eigen::MatrixXd::Identity(n,n)),
  K_(Eigen::MatrixXd::Zero(n,m)),
  P_(Eigen::MatrixXd::Identity(n,n)),
  Q_(Eigen::MatrixXd::Identity(n,n)), 
  R_(Eigen::MatrixXd::Identity(m,m)), 
  S_(Eigen::MatrixXd::Identity(m,m)),
  x_hat_(Eigen::VectorXd::Zero(n))
{}

void KalmanFilter::predict(const Eigen::VectorXd& u)
{
    x_hat_ = A_ * x_hat_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& y) 
{
    S_ = C_ * P_ * C_.transpose() + R_;
    K_ = P_ * C_.transpose() * S_.inverse();
    x_hat_ = x_hat_ + K_ * (y - C_ * x_hat_);
    P_ = (I_ - K_ * C_) * P_;
}