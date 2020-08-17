// Loosely based on kalman-cpp: https://github.com/hmartiro/kalman-cpp

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <Eigen/Dense>

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
        const Eigen::Matrix3f& A,
        const Eigen::Matrix3f& B,
        const Eigen::Matrix3f& C,
        const Eigen::Matrix3f& Q,
        const Eigen::Matrix3f& R,
        const Eigen::Matrix3f& P
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
    void init(double t0, Eigen::Vector3f x0);

    /**
     * Predict estimated state based on the time step, control matrix, and control input
     * Note that I am using B and an input because it varies with time for my system
     */
    void predict(double dt, const Eigen::Matrix3f& B, const Eigen::Vector3f& u);

    /**
     * Update the estimated state based on measured values.
     * Can pass in R to scale how much measurement error there will be based on situation (in my case, velocity)
     */
    void update(const Eigen::Vector3f& y, const Eigen::Matrix3f& R);

    /**
     * Return the current state and time.
     */
    Eigen::Vector3f state() { return x_hat; };
    double time() { return t; };
    Eigen::Matrix3f cov() {return P; };

  private:

    // Matrices for computation
    Eigen::Matrix3f A, B, C, Q, R, P, K, P0;

    // Initial and current time
    double t0, t;

    // n-size identity
    Eigen::Matrix3f I;

    // Estimated states
    Eigen::Vector3f x_hat, x_hat_new;
};

#endif
