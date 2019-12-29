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

void KalmanFilter::predict(double dt, const mat& B, const mat& u)
{
  this->B = B;
  this->dt = dt;
  x_hat_new = A * x_hat + B * u;
  P = A*P*A.t() + Q;
  t += dt;
  x_hat = x_hat_new;
}

void KalmanFilter::update(const mat& y, const mat& R, HardwareSerial& debug) 
{

  // Note print for a single column matrix doesn't work for some reason
  debug.println("State estimate before update");
  debug.print("[X: ");
  debug.print(x_hat(0,0), 4);
  debug.print(", Y: ");
  debug.print(x_hat(1,0), 4);
  debug.print(", A: ");
  debug.print(x_hat(2,0), 4);
  debug.println("]");

  this->R = R;
  K = P*C.t()*(C*P*C.t() + R).inv();
  x_hat_new += K * (y - C*x_hat);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  debug.println("State estimate after update");
  debug.print("[X: ");
  debug.print(x_hat(0,0), 4);
  debug.print(", Y: ");
  debug.print(x_hat(1,0), 4);
  debug.print(", A: ");
  debug.print(x_hat(2,0), 4);
  debug.println("]");

  String s1;
  debug.println("R matrix");
  R.print(s1);
  debug.println(s1);

  String s2;
  debug.println("P matrix");
  P.print(s2);
  debug.println(s2);

  String s3;
  debug.println("K matrix");
  K.print(s3);
  debug.println(s3);
}
