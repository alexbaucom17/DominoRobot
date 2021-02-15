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

    Eigen::Vector3f marvelmindToRobotFrame(Eigen::Vector3f mm_global_position);
    float computeVelocityUpdateFraction();
    
    // Current position and velocity
    Point pos_;
    Velocity vel_;

    // Parameters for localization algorithms
    float update_fraction_at_zero_vel_;
    float val_for_zero_update_;
    float mm_x_offset_;
    float mm_y_offset_;


};

#endif //Localization_h