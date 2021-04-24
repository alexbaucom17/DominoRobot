#include "CameraTracker.h"

#include <plog/Log.h>
#include "constants.h"


CameraTracker::CameraTracker(bool start_thread)
: camera_loop_time_averager_(10),
  loop_time_ms_(0)
  side_cam_(CAMERA_ID::SIDE, start_thread),
  rear_cam_(CAMERA_ID::REAR, start_thread)
{
    // Target points
    robot_P_side_target_ = {cfg.lookup("vision_tracker.physical.side.target_x"), 
                            cfg.lookup("vision_tracker.physical.side.target_y")};
    robot_P_rear_target_ = {cfg.lookup("vision_tracker.physical.rear.target_x"), 
                            cfg.lookup("vision_tracker.physical.rear.target_y")};
}



CameraTracker::~CameraTracker() {}

Point CameraTracker::getPoseFromCamera()
{
    std::lock_guard<std::mutex> read_lock(pose_mutex);
    return current_point_;
}

int CameraTracker::getLoopTimeMs()
{
    std::lock_guard<std::mutex> read_lock(loop_time_mutex);
    return loop_time_ms_;
}

void CameraTracker::start()
{
    if(!thread_running_)
    {
        PLOGI << "Starting camera thread";
        thread_ = std::thread(&CameraTracker::threadLoop, this);
        thread_.detach();
        thread_running_ = true;
    }
    std::lock_guard<std::mutex> read_lock(running_mutex);
    running_ = true;
}

void CameraTracker::stop()
{
    std::lock_guard<std::mutex> read_lock(running_mutex);
    running_ = false;
}



void CameraTracker::test_function()
{
    if (isRunning()) 
    {
        PLOGE << "Skipping test function while background thread is running";
        return;
    }
    
    cv::Mat frame;
    Timer t0;
    {
        Timer t1;
        if(!use_debug_image_) cameras_[CAMERA_ID::SIDE].capture >> frame;
        PLOGI << "Side camera frame took " << t1.dt_ms() << "ms to get";
        std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, CAMERA_ID::SIDE, true);
        cv::Point2f best_point_px = getBestKeypoint(keypoints);
        Eigen::Vector2f best_point_m = cameraToRobot(best_point_px, CAMERA_ID::SIDE);
        PLOGI << "Best keypoint side at " << best_point_px << " px";
        PLOGI << "Best keypoint side at " << best_point_m << " m";
        PLOGI << "Side camera took " << t1.dt_ms() << "ms to run";
    }
    {
        Timer t1;
        if(!use_debug_image_) cameras_[CAMERA_ID::REAR].capture >> frame;
        PLOGI << "Rear camera frame took " << t1.dt_ms() << "ms to get";
        std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, CAMERA_ID::REAR, true);
        cv::Point2f best_point_px = getBestKeypoint(keypoints);
        Eigen::Vector2f best_point_m = cameraToRobot(best_point_px, CAMERA_ID::REAR);
        PLOGI << "Best keypoint rear at " << best_point_px << " px";
        PLOGI << "Best keypoint rear at " << best_point_m << " m";
        PLOGI << "Rear camera took " << t1.dt_ms() << "ms to run";
    }

    PLOGI << "Done with image processing";
    PLOGI << "Total image processing time: " << t0.dt_ms() << "ms";
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

