#include "KalmanFilter.h"
#include "LinearAlgebra.h"


mat A = mat::identity(6);
mat B = mat::zeros(6,3);
mat C = mat::zeros(3,6);
mat Q = mat::identity(6);
mat R = mat::zeros(3,3);
mat P = mat::identity(6);
KalmanFilter kf = KalmanFilter(0.1, A, B, C, Q, R, P);

void setup()
{
    A(3,3) = 0;
    A(4,4) = 0;
    A(5,5) = 0;
    B(3,0) = 1;
    B(4,1) = 1;
    B(5,2) = 1;
    C(0,0) = 1;
    C(1,1) = 1;
    C(2,2) = 1;
    kf.init();
}


void loop()
{
    double dt = 0.1;
    A(0,3) = dt;
    A(1,4) = dt;
    A(2,6) = dt;
    mat u = mat::ones(1,3);

    mat y = mat::ones(1,3);

    kf.predict(dt, A, u);
    kf.update(y);

    mat x = kf.state();
    String s;
    x.print(s);

}
