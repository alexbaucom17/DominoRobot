#include "CameraTracker.h"

#include <plog/Log.h>
#include "constants.h"


CameraTracker::CameraTracker(bool start_thread)
: camera_loop_time_averager_(10),
  rear_cam_(CAMERA_ID::REAR, start_thread),
  side_cam_(CAMERA_ID::SIDE, start_thread),
  last_rear_cam_output_(),
  last_side_cam_output_()
{
    // Target points
    robot_P_side_target_ = {cfg.lookup("vision_tracker.physical.side.target_x"), 
                            cfg.lookup("vision_tracker.physical.side.target_y")};
    robot_P_rear_target_ = {cfg.lookup("vision_tracker.physical.rear.target_x"), 
                            cfg.lookup("vision_tracker.physical.rear.target_y")};
}

CameraTracker::~CameraTracker() {}

CameraTrackerOutput CameraTracker::getPoseFromCamera()
{
    CameraTrackerOutput output = {{0,0,0}, false};
    
    CameraPipelineOutput side_output = side_cam_.getData();
    CameraPipelineOutput rear_output = rear_cam_.getData();
    debug_.side_detection = side_output.ok;
    debug_.rear_detection = rear_output.ok;

    if(rear_output.ok)
    {
        last_rear_cam_output_ = rear_output;
    }
    if(side_output.ok)
    {
        last_side_cam_output_ = side_output;
    }

    if(!last_side_cam_output_.ok || !last_rear_cam_output_.ok) return output;

    int time_delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>
        (last_side_cam_output_.timestamp - last_rear_cam_output_.timestamp).count();
    if(abs(time_delta_ms) > 200)
    {
        PLOGW << "Camera timestamp delta too large: " << time_delta_ms;
        return output;
    }

    // Populate output
    output.pose = computeRobotPoseFromImagePoints(last_side_cam_output_.point, last_rear_cam_output_.point);
    output.ok = true;
    camera_loop_time_averager_.mark_point();

    // Populate debug
    debug_.side_u = last_side_cam_output_.uv[0];
    debug_.side_v = last_side_cam_output_.uv[1];
    debug_.rear_u = last_rear_cam_output_.uv[0];
    debug_.rear_v = last_rear_cam_output_.uv[1];
    debug_.side_x = last_side_cam_output_.point[0];
    debug_.side_y = last_side_cam_output_.point[1];
    debug_.rear_x = last_rear_cam_output_.point[0];
    debug_.rear_y = last_rear_cam_output_.point[1];
    debug_.pose_x = output.pose.x;
    debug_.pose_y = output.pose.y;
    debug_.pose_a = output.pose.a;
    debug_.loop_ms = camera_loop_time_averager_.get_ms();

    // Mark output values as not okay to make sure they are only used once
    last_rear_cam_output_.ok = false;
    last_side_cam_output_.ok = false;

    return output;
}

CameraDebug CameraTracker::getCameraDebug()
{
    return debug_;
}

void CameraTracker::start()
{
    rear_cam_.start();
    side_cam_.start();
}

void CameraTracker::stop()
{
    rear_cam_.stop();
    side_cam_.stop();
}


void CameraTracker::test_function()
{
    // if (isRunning()) 
    // {
    //     PLOGE << "Skipping test function while background thread is running";
    //     return;
    // }
    
    // cv::Mat frame;
    // Timer t0;
    // {
    //     Timer t1;
    //     if(!use_debug_image_) cameras_[CAMERA_ID::SIDE].capture >> frame;
    //     PLOGI << "Side camera frame took " << t1.dt_ms() << "ms to get";
    //     std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, CAMERA_ID::SIDE, true);
    //     cv::Point2f best_point_px = getBestKeypoint(keypoints);
    //     Eigen::Vector2f best_point_m = cameraToRobot(best_point_px, CAMERA_ID::SIDE);
    //     PLOGI << "Best keypoint side at " << best_point_px << " px";
    //     PLOGI << "Best keypoint side at " << best_point_m << " m";
    //     PLOGI << "Side camera took " << t1.dt_ms() << "ms to run";
    // }
    // {
    //     Timer t1;
    //     if(!use_debug_image_) cameras_[CAMERA_ID::REAR].capture >> frame;
    //     PLOGI << "Rear camera frame took " << t1.dt_ms() << "ms to get";
    //     std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, CAMERA_ID::REAR, true);
    //     cv::Point2f best_point_px = getBestKeypoint(keypoints);
    //     Eigen::Vector2f best_point_m = cameraToRobot(best_point_px, CAMERA_ID::REAR);
    //     PLOGI << "Best keypoint rear at " << best_point_px << " px";
    //     PLOGI << "Best keypoint rear at " << best_point_m << " m";
    //     PLOGI << "Rear camera took " << t1.dt_ms() << "ms to run";
    // }

    // PLOGI << "Done with image processing";
    // PLOGI << "Total image processing time: " << t0.dt_ms() << "ms";
}



Point CameraTracker::computeRobotPoseFromImagePoints(Eigen::Vector2f p_side, Eigen::Vector2f p_rear)
{  
    // Solve linear equations to get pose values
    Eigen::Matrix4f A;
    Eigen::Vector4f b;
    A << 1, 0, -p_rear[1], p_rear[0],
         0, 1,  p_rear[0], p_rear[1],
         1, 0, -p_side[1], p_side[0],
         0, 1,  p_side[0], p_side[1];
    b << robot_P_rear_target_[0],
         robot_P_rear_target_[1],
         robot_P_side_target_[0],
         robot_P_side_target_[1];
    Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);
    
    // Populate pose with angle averaging
    float pose_x = x[0];
    float pose_y = x[1];
    float pose_a = atan2(x[2], x[3]);
    return {pose_x, pose_y, pose_a};
}

