#include "Localization.h"

#include <math.h>
#include <algorithm> 
#include <plog/Log.h>
#include "constants.h"

Localization::Localization()
: pos_(0,0,0),
  vel_(0,0,0),
  update_fraction_at_zero_vel_(cfg.lookup("localization.update_fraction_at_zero_vel")),
  val_for_zero_update_(cfg.lookup("localization.val_for_zero_update")),
  mm_x_offset_(cfg.lookup("localization.mm_x_offset")),
  mm_y_offset_(cfg.lookup("localization.mm_y_offset")),
  position_reliability_zscore_thresh_(cfg.lookup("localization.position_reliability_zscore_thresh")),
  position_reliability_max_stddev_pos_(cfg.lookup("localization.position_reliability_max_stddev_pos")),
  position_reliability_max_stddev_ang_(cfg.lookup("localization.position_reliability_max_stddev_ang")),
  prev_positions_raw_(cfg.lookup("localization.position_reliability_buffer_size")),
  prev_positions_filtered_(cfg.lookup("localization.position_reliability_buffer_size")),
  metrics_(),
  last_valid_reading_timer_(),
  reading_validity_buffer_(cfg.lookup("localization.position_reliability_buffer_size"))
{}

void Localization::updatePositionReading(Point global_position)
{
    if (fabs(global_position.a) > 3.2) 
    {
        PLOGE << "INVALID ANGLE - should be in radians between +/- pi";
        return;
    }
    
    // The x,y,a here is from the center of the marvlemind pair, so we need to transform it to the actual center of the
    // robot (i.e. center of rotation)
    Eigen::Vector3f raw_measured_position = {global_position.x, global_position.y, global_position.a};
    Eigen::Vector3f adjusted_measured_position = marvelmindToRobotCenter(raw_measured_position);
    
    // Generate an update fraction based on the current velocity since we know the beacons are less accurate when moving
    const float update_fraction = computeVelocityUpdateFraction();
    const float reading_reliability = computePositionReadingReliability(adjusted_measured_position);
    updateMetricsForPosition(update_fraction, reading_reliability);
    
    // Actually update the position based on the observed position and the update fraction
    pos_.x += update_fraction * reading_reliability * (adjusted_measured_position(0) - pos_.x);
    pos_.y += update_fraction * reading_reliability * (adjusted_measured_position(1) - pos_.y);
    pos_.a = wrap_angle(pos_.a + update_fraction * reading_reliability * (angle_diff(adjusted_measured_position(2), pos_.a)));

    PLOGI_(LOCALIZATION_LOG_ID).printf("\nPosition update:");
    PLOGI_(LOCALIZATION_LOG_ID).printf("  Raw input: [%4.3f, %4.3f, %4.3f]", 
        raw_measured_position(0), raw_measured_position(1), raw_measured_position(2));
    PLOGI_(LOCALIZATION_LOG_ID).printf("  Adjusted input: [%4.3f, %4.3f, %4.3f]", 
        adjusted_measured_position(0), adjusted_measured_position(1), adjusted_measured_position(2));
    PLOGI_(LOCALIZATION_LOG_ID).printf("  vel_fraction: %4.3f, reliability: %4.3f", update_fraction, reading_reliability);
    PLOGI_(LOCALIZATION_LOG_ID).printf("  Current position: %s\n", pos_.toString().c_str());
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
    pos_.a = wrap_angle(pos_.a + vel_.va * dt);

    // Update metric confidence based on velocity reading
    Eigen::Vector3f v = {vel_.vx, vel_.vy, vel_.va};
    float total_v = v.norm();
    float delta_confidence = total_v * dt * 0.01;
    metrics_.confidence -= delta_confidence;
    metrics_.confidence = std::max(metrics_.confidence, 0.0f);
}

Eigen::Vector3f Localization::marvelmindToRobotCenter(Eigen::Vector3f mm_global_position) 
{
    Eigen::Vector3f local_offset = {mm_x_offset_/1000.0f, mm_y_offset_/1000.0f, 0.0f};
    float cA = cos(mm_global_position(2));
    float sA = sin(mm_global_position(2));
    Eigen::Matrix3f rotation;
    rotation << cA, -sA, 0.0f, 
                sA, cA, 0.0f,
                0.0f, 0.0f, 1.0f;
    Eigen::Vector3f adjusted_position = mm_global_position - rotation * local_offset;

    PLOGD_(LOCALIZATION_LOG_ID).printf("\nMM to robot frame:");
    PLOGD_(LOCALIZATION_LOG_ID).printf("  mm position: [%4.3f, %4.3f, %4.3f]", mm_global_position(0), mm_global_position(1), mm_global_position(2));
    PLOGD_(LOCALIZATION_LOG_ID).printf("  R: \n[%4.3f, %4.3f, %4.3f]\n[%4.3f, %4.3f, %4.3f]\n[%4.3f, %4.3f, %4.3f]", 
     rotation(0,0), rotation(0,1), rotation(0,2), rotation(1,0), rotation(1,1), rotation(1,2), rotation(2,0), rotation(2,1), rotation(2,2));
    PLOGD_(LOCALIZATION_LOG_ID).printf("  new position: [%4.3f, %4.3f, %4.3f]", adjusted_position(0), adjusted_position(1), adjusted_position(2));

    return adjusted_position;
}

float Localization::computeVelocityUpdateFraction()
{
    Eigen::Vector3f v = {vel_.vx, vel_.vy, vel_.va};
    float total_v = v.norm();
    float slope = update_fraction_at_zero_vel_ / -val_for_zero_update_;
    float update_fraction = update_fraction_at_zero_vel_ + slope * total_v;
    update_fraction = std::max(std::min(update_fraction, update_fraction_at_zero_vel_), 0.0f);

    PLOGD_(LOCALIZATION_LOG_ID).printf("\nVelocity fraction:");
    PLOGD_(LOCALIZATION_LOG_ID).printf("  Total v: %4.3f, slope: %4.3f, update_fraction: %4.3f", total_v, slope, update_fraction);

    return update_fraction;
}

float Localization::computePositionReadingReliability(Eigen::Vector3f position) 
{
    std::vector<Eigen::Vector3f> previous_readings = prev_positions_raw_.get_contents();
    prev_positions_raw_.insert(position);

    PLOGD_(LOCALIZATION_LOG_ID).printf("\nPosition reliability:");

    // Hacky algorithm to calculate standard deviation in each dimension
    // Partially borrowed from https://stackoverflow.com/questions/33268513/calculating-standard-deviation-variance-in-c
    bool all_valid = true;
    const int num_points = previous_readings.size();
    PLOGD_(LOCALIZATION_LOG_ID).printf("  Num points: %i", num_points);
    if(num_points > 3)
    {
        for(int i = 0; i < 3; i++) 
        {
            PLOGD_(LOCALIZATION_LOG_ID).printf("  Axis: %i", i);
            // Compute mean of previous samples
            float mean = 0;
            for (const auto& reading : previous_readings)
            {
                mean += reading(i);
            }
            mean /= num_points; 
            PLOGD_(LOCALIZATION_LOG_ID).printf("    Mean: %4.3f", mean);

            // Compute standard deviation of previous samples
            float variance = 0;
            for (const auto& reading : previous_readings)
            {
                variance += (reading(i) - mean) * (reading(i) - mean);
            }
            variance /= num_points;
            const float stddev = sqrt(variance);
            PLOGD_(LOCALIZATION_LOG_ID).printf("    Stddev: %4.3f", stddev);

            // If stddev of population is too high, don't consider it
            float max_stddev = (i < 2) ? position_reliability_max_stddev_pos_ : position_reliability_max_stddev_ang_;
            if (stddev > max_stddev)
            {
                all_valid = false;
                PLOGD_(LOCALIZATION_LOG_ID).printf("    Invalid stddev");
                break;
            }

            // Make sure we don't get a divide by zero error
            if (stddev < 0.0001) 
            {
                PLOGD_(LOCALIZATION_LOG_ID).printf("    Skipped zcore check due to small stddev");
                continue;
            }

            // Compute Z score of the current sample and check if it is in the range to keep
            const float z_score = fabs((position(i) - mean)/stddev);
            PLOGD_(LOCALIZATION_LOG_ID).printf("    Zscore: %4.3f", z_score);
            if(z_score > position_reliability_zscore_thresh_)
            {
                PLOGD_(LOCALIZATION_LOG_ID).printf("    Invalid zscore");
                all_valid = false;
                break;
            }
        }
    }
    else { PLOGD_(LOCALIZATION_LOG_ID).printf("  Skipped - not enough points");}

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

void Localization::updateMetricsForPosition(float update_fraction, float reading_reliability)
{
    float reading_validity = update_fraction * reading_reliability;
    if(reading_validity > 0) 
    {
        metrics_.last_reading_reliability = reading_reliability;
        metrics_.last_reading_update_fraction = update_fraction;
        metrics_.confidence = reading_validity;
        last_valid_reading_timer_.reset();
    }
    
    reading_validity_buffer_.insert(reading_validity);
    std::vector<float> buffer_contents = reading_validity_buffer_.get_contents();
    float mean = 0.0;
    for (const auto& val : buffer_contents) 
    {
        mean += val;
    }
    mean /= buffer_contents.size();

    metrics_.seconds_since_last_valid_reading = last_valid_reading_timer_.dt_s();
    metrics_.rolling_reading_filter_fraction = mean;
}