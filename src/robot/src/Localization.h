#ifndef Localization_h
#define Localization_h

#include "utils.h"
#include <Eigen/Dense>
#include "KalmanFilter.h"

class Localization
{
  public:
    Localization();

    void updatePositionReading(Point global_position);

    void updateVelocityReading(Velocity local_cart_vel, float dt);

    Point getPosition() {return pos_; };

    Velocity getVelocity() {return vel_; };

    void forceZeroVelocity() {vel_ = {0,0,0}; };
    
    // Force the position to a specific value, bypassing localization algorithms (used for testing/debugging)
    void forceSetPosition(Point global_position) {pos_ = global_position;};

    LocalizationMetrics getLocalizationMetrics() { return metrics_; };

    void resetAngleCovariance();

  private:

    // Convert position reading from marvelmind frame to robot frame
    Eigen::Vector3f marvelmindToRobotCenter(Eigen::Vector3f mm_global_position);

    // Compute a multiplier based on the current velocity that informs how trustworthy the current reading might be
    float computePositionUncertainty();

    
    // Current position and velocity
    Point pos_;
    Velocity vel_;
    
    // Parameters for localization algorithms
    float mm_x_offset_;
    float mm_y_offset_;
    float variance_ref_trans_;
    float variance_ref_angle_;
    float meas_trans_cov_;
    float meas_angle_cov_;
    float localization_uncertainty_scale_;
    float min_vel_uncertainty_;
    float vel_uncertainty_slope_;
    float max_vel_uncetainty_;
    float vel_uncertainty_decay_time_;

    LocalizationMetrics metrics_;
    Timer time_since_last_motion_; 
    KalmanFilter kf_;
};

#endif //Localization_h