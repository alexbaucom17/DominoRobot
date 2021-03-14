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
  filtered_positions_mean_(),
  filtered_positions_stddev_(),
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
    updateBuffersForPositionReading(adjusted_measured_position);
    const float reading_reliability = computePositionReliability();
    updateMetricsForPosition(update_fraction, reading_reliability);
    
    // Update based on the mean - but should use the update fraction and maybe a time decay?
    pos_.x = filtered_positions_mean_[0];
    pos_.y = filtered_positions_mean_[1];
    pos_.a = wrap_angle(filtered_positions_mean_[2]);

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

void Localization::updateBuffersForPositionReading(Eigen::Vector3f position)
{
    prev_positions_raw_.insert(position);
    std::vector<Eigen::Vector3f> previous_readings = prev_positions_raw_.get_contents();
    PLOGD_(LOCALIZATION_LOG_ID).printf("\nPosition reliability:");
    PLOGD_(LOCALIZATION_LOG_ID).printf("  Num points: %i", previous_readings.size());

    if(previous_readings.size() < 3)
    {
        PLOGD_(LOCALIZATION_LOG_ID).printf("  Skipped - not enough points");
        return;
    }

    std::vector<float> all_means;
    all_means.reserve(3);
    std::vector<float> all_stddevs;
    all_stddevs.reserve(3);
    std::vector<float> all_zscores;
    all_zscores.reserve(3);

    bool all_valid = true;
    for (int i = 0; i < 3; i++)
    {
        // Convert from vector of poses over time to time vectors of component
        std::vector<float> readings;
        readings.reserve(previous_readings.size());
        for (uint j = 0; j < previous_readings.size(); j++) 
        {
            readings.push_back(previous_readings[j](i));
        }

        // Compute various metrics
        float mean = vectorMean(readings);
        float stddev = vectorStddev(readings, mean);
        float zscore = zScore(mean, stddev, position(i));
        all_means.push_back(mean);
        all_stddevs.push_back(stddev);
        all_zscores.push_back(zscore);

        // Change stddev limit for translation and rotation
        float max_stddev = (i < 2) ? position_reliability_max_stddev_pos_ : position_reliability_max_stddev_ang_;
        if(stddev > max_stddev || zscore > position_reliability_zscore_thresh_ )
        {
            all_valid = false;
            PLOGD_(LOCALIZATION_LOG_ID).printf("    Invalid point due to stddev or zscore");
        }
        PLOGD_(LOCALIZATION_LOG_ID).printf("  Axis: %i", i);
        PLOGD_(LOCALIZATION_LOG_ID).printf("    Mean: %4.3f", mean);
        PLOGD_(LOCALIZATION_LOG_ID).printf("    Stddev: %4.3f", stddev);
        PLOGD_(LOCALIZATION_LOG_ID).printf("    Zscore: %4.3f", zscore);
    }

    if(all_valid)
    {
        prev_positions_filtered_.insert(position);
    }
}

// TODO: add time decay and weighting for previous positions
float Localization::computePositionReliability()
{
    std::vector<Eigen::Vector3f> previous_filtered_readings = prev_positions_filtered_.get_contents();
    std::vector<float> all_means;
    all_means.reserve(3);
    std::vector<float> all_stddevs;
    all_stddevs.reserve(3);
    float max_stddev = 0;
    for (int i = 0; i < 3; i++)
    {
        // Convert from vector of poses over time to time vectors of component
        std::vector<float> readings;
        readings.reserve(previous_filtered_readings.size());
        for (uint j = 0; j < previous_filtered_readings.size(); j++) 
        {
            readings.push_back(previous_filtered_readings[j](i));
        }
        float mean = vectorMean(readings);
        float stddev = vectorStddev(readings, mean);

        all_means.push_back(mean);
        all_stddevs.push_back(stddev);
        if (stddev > max_stddev)
        {
            max_stddev = stddev;
        }
    }
    filtered_positions_mean_ = all_means;
    filtered_positions_stddev_ = all_stddevs;

    // Compute reliability score as worst stddev compared to threshold
    return 1.0 - max_stddev/position_reliability_max_stddev_pos_;

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