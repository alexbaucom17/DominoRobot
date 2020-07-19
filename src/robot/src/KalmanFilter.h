// Loosely based on kalman-cpp: https://github.com/hmartiro/kalman-cpp

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <LinearAlgebra.h>
#include <HardwareSerial.h>

class KalmanFilter 
{

  public:

    /**
     * Create a Kalman filter with the specified matrices.
     *   A - System dynamics matrix
     *   B - Control matrix
     *   C - Output matrix
     *   Q - Process noise covariance
     *   R - Measurement noise covariance
     *   P - Estimate error covariance
     */
    KalmanFilter(
        double dt,
        const mat& A,
        const mat& B,
        const mat& C,
        const mat& Q,
        const mat& R,
        const mat& P
    );

    /**
     * Empty constructor
     */
    KalmanFilter();

    /**
     * Initialize the filter with initial states as zero.
     */
    void init();

    /**
     * Initialize the filter with a guess for initial states.
     */
    void init(double t0, mat x0);

    /**
     * Predict estimated state based on the time step, control matrix, and control input
     * Note that I am using B and an input because it varies with time for my system
     */
    void predict(double dt, const mat& B, const mat& u);

    /**
     * Update the estimated state based on measured values.
     * Can pass in R to scale how much measurement error there will be based on situation (in my case, velocity)
     */
    void update(const mat& y, const mat& R, HardwareSerial& debug);

    /**
     * Return the current state and time.
     */
    mat state() { return x_hat; };
    double time() { return t; };
    mat cov() {return P; };

  private:

    // Matrices for computation
    mat A, B, C, Q, R, P, K, P0;

    // System dimensions
    uint8_t m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // n-size identity
    mat I;

    // Estimated states
    mat x_hat, x_hat_new;
};

#endif
