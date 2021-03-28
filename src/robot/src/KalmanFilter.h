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
        const Eigen::MatrixXf& A,
        const Eigen::MatrixXf& B,
        const Eigen::MatrixXf& C,
        const Eigen::MatrixXf& Q,
        const Eigen::MatrixXf& R
    );

    // Size only simplify initializing in a class
    KalmanFilter(int n, int m);

    // Prediction step with input u
    void predict(const Eigen::VectorXf& u);

    // Update the estimated state based on measured values.
    void update(const Eigen::VectorXf& y);

    // Get current state estimate
    Eigen::VectorXf state() { return x_hat_; };
    Eigen::MatrixXf covariance() { return P_; };

  private:

    // System dimensions
    int n_;
    int m_;

    // Matrices for computation
    Eigen::MatrixXf A_;
    Eigen::MatrixXf B_;
    Eigen::MatrixXf C_;
    Eigen::MatrixXf I_;
    Eigen::MatrixXf K_;
    Eigen::MatrixXf P_;
    Eigen::MatrixXf Q_;
    Eigen::MatrixXf R_;
    Eigen::MatrixXf S_;

    // Estimated state
    Eigen::VectorXf x_hat_;
};

#endif //KalmanFilter_h