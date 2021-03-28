#include "KalmanFilter.h"

KalmanFilter(
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
  A_(MatrixXd::Identity(n,n)), 
  B_(MatrixXd::Zero(n,1)), 
  C_(MatrixXd::Zero(m,n)), 
  K_(MatrixXd::Zero(n,m))
  P_(MatrixXd::Identity(n))
  Q_(MatrixXd::Identity(n)), 
  R_(MatrixXd::Identity(m)), 
  S_(MatrixXd::Identity(m))
  I_(MatrixXd::Identity(n)),
  x_hat_(VectorXd::Zero(n))
{}

void predict(const Eigen::VectorXd& u)
{
    x_hat_ = A_ * x_hat_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& y) 
{
    S_ = C_ * P_ * C_.transpose() + R_;
    K_ = P_ * C_.transpose() * S_.inverse();
    x_hat_ = x_hat_ + K_ * (y_ - C_ * x_hat_);
    P_ = (I_ - K_ * C_) * P_;
}