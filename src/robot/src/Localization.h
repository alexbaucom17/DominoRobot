#ifndef Localization_h
#define Localization_h

#include "utils.h"
#include <Eigen/Dense>
#include "kalman_filters/kalman_filter.h"

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

  private:

    // Convert position reading from marvelmind frame to robot frame
    Eigen::Vector3f marvelmindToRobotCenter(Eigen::Vector3f mm_global_position);

    // Compute a multiplier based on the current velocity that informs how trustworthy the current reading might be
    float computeVelocityUpdateFraction();

    // Compute a position reliability multiplier based on previously observed readings
    float computePositionReliability();

    // Update internal buffers with the new position
    void updateBuffersForPositionReading(Eigen::Vector3f position);

    // Update localization metrics based on latest position reading
    void updateMetricsForPosition(float update_fraction, float reading_reliability);
    
    // Current position and velocity
    Point pos_;
    Velocity vel_;
    
    // Parameters for localization algorithms
    float update_fraction_at_zero_vel_;
    float val_for_zero_update_;
    float mm_x_offset_;
    float mm_y_offset_;
    float position_reliability_zscore_thresh_;
    float position_reliability_max_stddev_pos_;
    float position_reliability_max_stddev_ang_;

    CircularBuffer<Eigen::Vector3f> prev_positions_raw_;
    CircularBuffer<Eigen::Vector3f> prev_positions_filtered_;
    std::vector<float> filtered_positions_mean_;
    std::vector<float> filtered_positions_stddev_;
    LocalizationMetrics metrics_;
    Timer last_valid_reading_timer_; 
    CircularBuffer<float> reading_validity_buffer_;
    bool use_kf_;
    arma::mat A_;
    arma::mat B_;
    arma::mat C_;
    arma::mat Q_;
    arma::mat R_;
    kf::KalmanFilter kf_;


};

#endif //Localization_h