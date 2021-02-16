#ifndef Localization_h
#define Localization_h

#include "utils.h"
#include <Eigen/Dense>

class Localization
{
  public:
    Localization();

    void updatePositionReading(Point global_position);

    void updateVelocityReading(Velocity local_cart_vel, float dt);

    Point getPosition() {return pos_; };

    Velocity getVelocity() {return vel_; };

    void forceZeroVelocity() {vel_ = {0,0,0}; };

  private:

    // Convert position reading from marvelmind frame to robot frame
    Eigen::Vector3f marvelmindToRobotFrame(Eigen::Vector3f mm_global_position);

    // Compute a multiplier based on the current velocity that informs how trustworthy the current reading might be
    float computeVelocityUpdateFraction();

    // Compute a position reliability multiplier based on previously observed readings
    float computePositionReadingReliability(Eigen::Vector3f position);
    
    // Current position and velocity
    Point pos_;
    Velocity vel_;

    // Parameters for localization algorithms
    float update_fraction_at_zero_vel_;
    float val_for_zero_update_;
    float mm_x_offset_;
    float mm_y_offset_;
    float position_reliability_zscore_thresh_;
    float position_reliability_max_stddev_;

    CircularBuffer<Eigen::Vector3f> prev_positions_raw_;
    CircularBuffer<Eigen::Vector3f> prev_positions_filtered_;


};

#endif //Localization_h