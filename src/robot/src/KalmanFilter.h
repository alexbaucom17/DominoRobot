/**
* Kalman filter loosely based on
* https://github.com/hmartiro/kalman-cpp
* and
* https://github.com/tysik/kalman_filters
*/

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <Eigen/Dense>

class KalmanFilter 
{

  public:

    KalmanFilter(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R
    );

    // Size only simplify initializing in a class
    KalmanFilter(int n, int m);

    // Prediction step with input u
    void predict(const Eigen::VectorXd& u);

    // Update the estimated state based on measured values.
    void update(const Eigen::VectorXd& y);

    // Get current state estimate
    Eigen::VectorXd state() { return x_hat_; };
    Eigen::MatrixXd covariance() { return P_; };

  private:

    // System dimensions
    int n_;
    int m_;

    // Matrices for computation
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd I_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd S_;

    // Estimated state
    Eigen::VectorXd x_hat_;
};

#endif //KalmanFilter_h