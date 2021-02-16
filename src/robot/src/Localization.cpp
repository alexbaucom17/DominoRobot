#include "Localization.h"

#include <math.h>
#include "constants.h"

Localization::Localization()
: pos_(0,0,0),
  vel_(0,0,0),
  update_fraction_at_zero_vel_(cfg.lookup("localization.update_fraction_at_zero_vel")),
  val_for_zero_update_(cfg.lookup("localization.val_for_zero_update")),
  mm_x_offset_(cfg.lookup("localization.mm_x_offset")),
  mm_y_offset_(cfg.lookup("localization.mm_y_offset")),
  position_reliability_stddev_thresh_(cfg.lookup("localization.position_reliability_stddev_thresh")),
  prev_positions_raw_(cfg.lookup("localization.position_reliability_buffer_size")),
  prev_positions_filtered_(cfg.lookup("localization.position_reliability_buffer_size"))
{}

void Localization::updatePositionReading(Point global_position)
{
    // The x,y,a here is from the center of the marvlemind pair, so we need to transform it to the actual center of the
    // robot (i.e. center of rotation)
    Eigen::Vector3f raw_measured_position = {global_position.x, global_position.y, global_position.a};
    Eigen::Vector3f adjusted_measured_position = marvelmindToRobotFrame(raw_measured_position);
    
    // Generate an update fraction based on the current velocity since we know the beacons are less accurate when moving
    const float update_fraction = computeVelocityUpdateFraction();
    const float reading_reliability = computePositionReadingReliability(adjusted_measured_position);
    
    // Actually update the position based on the observed position and the update fraction
    pos_.x += update_fraction * reading_reliability * (adjusted_measured_position(0) - pos_.x);
    pos_.y += update_fraction * reading_reliability * (adjusted_measured_position(1) - pos_.y);
    pos_.a += update_fraction * reading_reliability * (adjusted_measured_position(2) - pos_.a);

}

void Localization::updateVelocityReading(Velocity local_cart_vel, float dt)
{
    // Convert local cartesian velocity to global cartesian velocity using the last estimated angle
    float cA = cos(pos_.a);
    float sA = sin(pos_.a);
    vel_.vx = cA * local_cart_vel.vx - sA * local_cart_vel.vy;
    vel_.vy = sA * local_cart_vel.vx + cA * local_cart_vel.vy;
    vel_.va = local_cart_vel.va;

    // Compute new position estimate
    pos_.x += vel_.vx * dt;
    pos_.y += vel_.vy * dt;
    pos_.a += vel_.va * dt;
}

Eigen::Vector3f Localization::marvelmindToRobotFrame(Eigen::Vector3f mm_global_position) 
{
    Eigen::Vector3f local_offset = {mm_x_offset_/1000.0f, mm_y_offset_/1000.0f, 0.0f};
    float cA = cos(mm_global_position(2));
    float sA = sin(mm_global_position(2));
    Eigen::Matrix3f rotation;
    rotation << cA, -sA, 0.0f, 
                sA, cA, 0.0f,
                0.0f, 0.0f, 1.0f;
    Eigen::Vector3f adjusted_position = mm_global_position - rotation * local_offset;
    return adjusted_position;
}

float Localization::computeVelocityUpdateFraction()
{
    Eigen::Vector3f v = {vel_.vx, vel_.vy, vel_.va};
    float total_v = v.norm();
    float slope = update_fraction_at_zero_vel_ / -val_for_zero_update_;
    float update_fraction = update_fraction_at_zero_vel_ + slope * total_v;
    update_fraction = std::max(std::min(update_fraction, update_fraction_at_zero_vel_), 0.0f);
    return update_fraction;
}

float Localization::computePositionReadingReliability(Eigen::Vector3f position) 
{
    std::vector<Eigen::Vector3f> previous_readings = prev_positions_raw_.get_contents();
    prev_positions_raw_.insert(position);

    // Hacky algorithm to calculate standard deviation in each dimension
    // Partially borrowed from https://stackoverflow.com/questions/33268513/calculating-standard-deviation-variance-in-c
    bool all_valid = true;
    const int num_points = previous_readings.size();
    for(int i = 0; i < 3; i++) 
    {
        float mean = 0;
        for (const auto& reading : previous_readings)
        {
            mean += reading(i);
        }
        mean /= num_points; 

        float variance = 0;
        for (const auto& reading : previous_readings)
        {
            variance += (reading(i) - mean) * (reading(i) - mean);
        }
        variance /= num_points;
        const float stddev = sqrt(variance);

        if(stddev > position_reliability_stddev_thresh_)
        {
            all_valid = false;
            break;
        }
    }

    // TODO: Make this more intelligent
    if(all_valid)
    {
        prev_positions_filtered_.insert(position);
        return 1.0;
    }
    else
    {
        return 0.0;
    }
}