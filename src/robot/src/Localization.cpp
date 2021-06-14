#include "Localization.h"

#include <math.h>
#include <algorithm> 
#include <plog/Log.h>
#include "constants.h"

Localization::Localization()
: pos_(0,0,0),
  vel_(0,0,0),
  mm_x_offset_(cfg.lookup("localization.mm_x_offset")),
  mm_y_offset_(cfg.lookup("localization.mm_y_offset")),
  variance_ref_trans_(cfg.lookup("localization.variance_ref_trans")),
  variance_ref_angle_(cfg.lookup("localization.variance_ref_angle")),
  meas_trans_cov_(cfg.lookup("localization.kf_meas_trans_cov")),
  meas_angle_cov_(cfg.lookup("localization.kf_meas_angle_cov")),
  localization_uncertainty_scale_(cfg.lookup("localization.kf_uncertainty_scale")),
  min_vel_uncertainty_(cfg.lookup("localization.min_vel_uncertainty")),
  vel_uncertainty_slope_(cfg.lookup("localization.vel_uncertainty_slope")),
  max_vel_uncetainty_(cfg.lookup("localization.max_vel_uncetainty")),
  vel_uncertainty_decay_time_(cfg.lookup("localization.vel_uncertainty_decay_time")),
  metrics_(),
  time_since_last_motion_(),
  kf_(3,3)
{
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf B = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf C = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf Q(3,3);
    float predict_trans_cov = cfg.lookup("localization.kf_predict_trans_cov");
    float predict_angle_cov = cfg.lookup("localization.kf_predict_angle_cov");
    Q << predict_trans_cov,0,0, 
         0,predict_trans_cov,0,
         0,0,predict_angle_cov;
    Eigen::MatrixXf R(3,3);
    R << meas_trans_cov_,0,0, 
         0,meas_trans_cov_,0,
         0,0,meas_angle_cov_;
    kf_ = KalmanFilter(A,B,C,Q,R);
}

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
    
    // Generate an uncertainty based on the current velocity and time since motion since we know the beacons are less accurate when moving
    const float position_uncertainty = computePositionUncertainty();
    metrics_.last_position_uncertainty = position_uncertainty;
    Eigen::MatrixXf R(3,3);
    R << meas_trans_cov_ + position_uncertainty*localization_uncertainty_scale_,0,0, 
         0,meas_trans_cov_+ position_uncertainty*localization_uncertainty_scale_,0,
         0,0,meas_angle_cov_+ position_uncertainty*localization_uncertainty_scale_;

    // PLOGI << "position_uncertainty: " << position_uncertainty;
    // PLOGI << "R: " << R;
    // PLOGI << "cov1: " << kf_.covariance();

    kf_.update(adjusted_measured_position, R);
    Eigen::VectorXf est = kf_.state();
    pos_.x = est[0];
    pos_.y = est[1];
    pos_.a = wrap_angle(est[2]);
    
    // PLOGI << "cov2: " << kf_.covariance();

    PLOGI_(LOCALIZATION_LOG_ID).printf("\nPosition update:");
    PLOGI_(LOCALIZATION_LOG_ID).printf("  Raw input: [%4.3f, %4.3f, %4.3f]", 
        raw_measured_position(0), raw_measured_position(1), raw_measured_position(2));
    PLOGI_(LOCALIZATION_LOG_ID).printf("  Adjusted input: [%4.3f, %4.3f, %4.3f]", 
        adjusted_measured_position(0), adjusted_measured_position(1), adjusted_measured_position(2));
    PLOGI_(LOCALIZATION_LOG_ID).printf("  position_uncertainty: %4.3f", position_uncertainty);
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

    // Apply prediction step with velocity to get estimated position
    Eigen::Vector3f u = {vel_.vx, vel_.vy, vel_.va};
    Eigen::Vector3f udt = dt*u;
    kf_.predict(udt);
    // PLOGI << "cov3: " << kf_.covariance();
    time_since_last_motion_.reset();

    Eigen::VectorXf est = kf_.state();
    pos_.x = est[0];
    pos_.y = est[1];
    pos_.a = wrap_angle(est[2]);

    // Using the covariance matrix from kf, using those values to estimate a fractional 'confidence'
    // in our positioning relative to some reference amout.
    Eigen::MatrixXf cov = kf_.covariance();

    // Compute inverse confidence. As long as the variance isn't larger than the reference values
    // these will be between 0-1 with 0 being more confident in the positioning
    metrics_.confidence_x = std::max(1-cov(0,0)/variance_ref_trans_, 0.0f);
    metrics_.confidence_y = std::max(1-cov(1,1)/variance_ref_trans_, 0.0f);
    metrics_.confidence_a = std::max(1-cov(2,2)/variance_ref_angle_, 0.0f);
    metrics_.total_confidence = (metrics_.confidence_x + metrics_.confidence_y + metrics_.confidence_a)/3;
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

    // PLOGD_(LOCALIZATION_LOG_ID).printf("\nMM to robot frame:");
    // PLOGD_(LOCALIZATION_LOG_ID).printf("  mm position: [%4.3f, %4.3f, %4.3f]", mm_global_position(0), mm_global_position(1), mm_global_position(2));
    // PLOGD_(LOCALIZATION_LOG_ID).printf("  R: \n[%4.3f, %4.3f, %4.3f]\n[%4.3f, %4.3f, %4.3f]\n[%4.3f, %4.3f, %4.3f]", 
    //  rotation(0,0), rotation(0,1), rotation(0,2), rotation(1,0), rotation(1,1), rotation(1,2), rotation(2,0), rotation(2,1), rotation(2,2));
    // PLOGD_(LOCALIZATION_LOG_ID).printf("  new position: [%4.3f, %4.3f, %4.3f]", adjusted_position(0), adjusted_position(1), adjusted_position(2));

    return adjusted_position;
}

float Localization::computePositionUncertainty()
{
    Eigen::Vector3f v = {vel_.vx, vel_.vy, vel_.va};
    float total_v = v.norm();

    float position_uncertainty = 0.0f;
    if(total_v > 0.01)
    {
        position_uncertainty = std::max(min_vel_uncertainty_ + vel_uncertainty_slope_*total_v, max_vel_uncetainty_);
    }
    else 
    {
        position_uncertainty = std::max(min_vel_uncertainty_ * (vel_uncertainty_decay_time_ - time_since_last_motion_.dt_s()) / vel_uncertainty_decay_time_, 0.0f);
    }

    PLOGD_(LOCALIZATION_LOG_ID).printf("\nPosition Uncertainty:");
    PLOGD_(LOCALIZATION_LOG_ID).printf("  Total v: %4.3f, time since motion: %4.3f, position_uncertainty: %4.3f", total_v, time_since_last_motion_.dt_s(), position_uncertainty);

    return position_uncertainty;
}