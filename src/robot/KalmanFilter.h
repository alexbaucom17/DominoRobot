// Loosely based on kalman-cpp: https://github.com/hmartiro/kalman-cpp

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <LinearAlgebra.h>

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
    void init(double t0, const mat& x0);

    /**
     * Predict estimated state based on the time step, dynamics matrix, and control input
     */
    void predict(double dt, const mat& A, const mat& u);

    /**
     * Update the estimated state based on measured values.
     */
    void update(const mat& y);

    /**
     * Return the current state and time.
     */
    mat state() { return x_hat; };
    double time() { return t; };

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
