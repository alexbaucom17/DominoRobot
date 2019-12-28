#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(
    double dt,
    const mat& A,
    const mat& B,
    const mat& C,
    const mat& Q,
    const mat& R,
    const mat& P)
  : A(A), B(B), C(C), Q(Q), R(R), P0(P),
    m(C.get_rows()), n(A.get_rows()), dt(dt),
    I(mat::identity(n)), x_hat(1,n), x_hat_new(1,n)
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
  x_hat = mat::zeros(n,1);
  P = P0;
  t0 = 0;
  t = t0;
}

void KalmanFilter::predict(double dt, const mat& A, const mat& u)
{
  this->A = A;
  this->dt = dt;
  x_hat_new = A * x_hat + B * u;
  P = A*P*A.t() + Q;
  t += dt;
  x_hat = x_hat_new;
}

void KalmanFilter::update(const mat& y) 
{
  K = P*C.t()*(C*P*C.t() + R).inv();
  x_hat_new += K * (y - C*x_hat);
  P = (I - K*C)*P;
  x_hat = x_hat_new;
}
