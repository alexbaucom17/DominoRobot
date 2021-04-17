#include "CameraTracker.h"

#include <plog/Log.h>
#include "constants.h"
#include <mutex>

std::mutex pose_mutex;
std::mutex loop_time_mutex;
std::mutex running_mutex;

cv::Point2f getBestKeypoint(std::vector<cv::KeyPoint> keypoints)
{
    if(keypoints.empty())
    {
        return {0,0};
    }
    
    // Get keypoint with largest area
    cv::KeyPoint best_keypoint = keypoints.front();
    for (const auto& k : keypoints) {
        // PLOGI << "Keypoint: " << k.class_id;
        // PLOGI << "Point: " << k.pt;
        // PLOGI << "Angle: " << k.angle;
        // PLOGI << "Size: " << k.size;
        if(k.size > best_keypoint.size)
        {
            best_keypoint = k;
        }
    }
    return best_keypoint.pt;
}


CameraTracker::CameraTracker(bool start_thread)
: use_debug_image_(cfg.lookup("vision_tracker.debug.use_debug_image")),
  output_debug_images_(cfg.lookup("vision_tracker.debug.save_camera_debug")),
  running_(false),
  thread_running_(false),
  current_point_({0,0,0}),
  camera_loop_time_averager_(10),
  loop_time_ms_(0),
  pixels_per_meter_u_(cfg.lookup("vision_tracker.physical.pixels_per_meter_u")),
  pixels_per_meter_v_(cfg.lookup("vision_tracker.physical.pixels_per_meter_v"))
{
    // Initialze cameras
    initCamera(CAMERA_ID::REAR);
    initCamera(CAMERA_ID::SIDE);

    // Configure detection parameters
    threshold_ = cfg.lookup("vision_tracker.detection.threshold");
    blob_params_.minThreshold = 10;
    blob_params_.maxThreshold = 200;

    blob_params_.filterByArea = cfg.lookup("vision_tracker.detection.blob.use_area");
    blob_params_.minArea = cfg.lookup("vision_tracker.detection.blob.min_area");
    blob_params_.maxArea = cfg.lookup("vision_tracker.detection.blob.max_area");
    blob_params_.filterByColor = false;
    blob_params_.filterByCircularity = cfg.lookup("vision_tracker.detection.blob.use_circularity");
    blob_params_.minCircularity = cfg.lookup("vision_tracker.detection.blob.min_circularity");
    blob_params_.maxCircularity = cfg.lookup("vision_tracker.detection.blob.max_circularity");
    blob_params_.filterByConvexity = false;
    blob_params_.filterByInertia = false;

    // Target points
    robot_P_side_target_ = {cfg.lookup("vision_tracker.physical.side.target_x"), 
                            cfg.lookup("vision_tracker.physical.side.target_y")};
    robot_P_rear_target_ = {cfg.lookup("vision_tracker.physical.rear.target_x"), 
                            cfg.lookup("vision_tracker.physical.rear.target_y")};

    // Start thread
    if (start_thread)
    {
        PLOGI << "Starting camera thread";
        thread_ = std::thread(&CameraTracker::threadLoop, this);
        thread_.detach();
        thread_running_ = true;
    }
}

void CameraTracker::initCamera(CAMERA_ID id)
{
    cameras_[id] = CameraData();
    CameraData& camera_data = cameras_[id];

    // Get config strings
    std::string name = cameraIdToString(id);
    std::string calibration_path_config_name = "vision_tracker." + name + ".calibration_file";
    std::string camera_path_config_name = "vision_tracker." + name + ".camera_path";
    std::string debug_output_path_config_name = "vision_tracker." + name + ".debug_output_path";
    std::string debug_image_config_name = "vision_tracker." + name + ".debug_image";
    std::string x_offset_config_name = "vision_tracker.physical." + name + ".x_offset";
    std::string y_offset_config_name = "vision_tracker.physical." + name + ".y_offset";
    std::string z_offset_config_name = "vision_tracker.physical." + name + ".z_offset";
    
    // Initialize intrinsic calibration data
    std::string calibration_path = cfg.lookup(calibration_path_config_name);
    PLOGI.printf("Loading %s camera calibration from %s", name.c_str(), calibration_path.c_str());
    cv::FileStorage fs(calibration_path, cv::FileStorage::READ);
    if(fs["K"].isNone() || fs["D"].isNone()) 
    {
        PLOGE.printf("Missing %s calibration data", name.c_str());
        throw;
    }
    fs["K"] >> camera_data.K;
    fs["D"] >> camera_data.D;

    // Initialize extrinsic data
    float x_offset = cfg.lookup(x_offset_config_name);
    float y_offset = cfg.lookup(y_offset_config_name);
    float z_offset = cfg.lookup(z_offset_config_name);
    Eigen::Vector3f camera_pose = {x_offset, y_offset, z_offset};
    Eigen::Matrix3f camera_rotation;
    // Hardcoding rotation matrices here for simplicity
    if(id == CAMERA_ID::SIDE) 
    {
        camera_rotation << -1,0,0, 0,1,0, 0,0,-1;
    }
    else
    {
        camera_rotation << 0,1,0,  -1,0,0, 0,0,-1;
    }
    // From https://ksimek.github.io/2012/08/22/extrinsic/
    camera_data.t = -1 * camera_rotation.transpose() * camera_pose;
    camera_data.R = camera_rotation.transpose();

    // Precompute some inverse values for later re-use
    camera_data.R_inv = camera_data.R.inverse();
    Eigen::Matrix3f tmp;
    // Hack to get around problems with enabling eigen support in opencv
    tmp << camera_data.K.at<double>(0,0), camera_data.K.at<double>(0,1), camera_data.K.at<double>(0,2),
           camera_data.K.at<double>(1,0), camera_data.K.at<double>(1,1), camera_data.K.at<double>(1,2),
           camera_data.K.at<double>(2,0), camera_data.K.at<double>(2,1), camera_data.K.at<double>(2,2);
    camera_data.K_inv = tmp.inverse();
    
    // Initialize camera calibration capture or debug data
    if(!use_debug_image_)
    {
        std::string camera_path = cfg.lookup(camera_path_config_name);
        camera_data.capture = cv::VideoCapture(camera_path);
        if (!camera_data.capture.isOpened()) 
        {
            PLOGE.printf("Could not open %s camera at %s", name.c_str(), camera_path.c_str());
            throw;
        }
        PLOGI.printf("Opened %s camera at %s", name.c_str(), camera_path.c_str());
    } 
    else 
    {
        std::string image_path = cfg.lookup(debug_image_config_name);
        camera_data.debug_frame = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if(camera_data.debug_frame.empty())
        {
            PLOGE.printf("Could not read %s debug image from %s", name.c_str(), image_path.c_str());
            throw;
        }
        PLOGW.printf("Loading %s debug image from %s", name.c_str(), image_path.c_str());
    }

    // Initialze debug output path
    if(output_debug_images_)
    {
        camera_data.debug_output_path = std::string(cfg.lookup(debug_output_path_config_name)); 
    }
}

std::string CameraTracker::cameraIdToString(CAMERA_ID id)
{
    if(id == CAMERA_ID::REAR) return "rear";
    if(id == CAMERA_ID::SIDE) return "side";
    PLOGE << "Unknown camera id";
    return "unk";
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

std::vector<cv::KeyPoint> CameraTracker::allKeypointsInImage(cv::Mat img_raw, CAMERA_ID id, bool output_debug)
{   
    // Undistort and crop
    cv::Mat img_undistorted;
    cv::Rect validPixROI;
    cv::Mat newcameramtx = cv::getOptimalNewCameraMatrix(cameras_[id].K, cameras_[id].D, img_raw.size(), /*alpha=*/1, img_raw.size(), &validPixROI);
    cv::undistort(img_raw, img_undistorted, cameras_[id].K, cameras_[id].D);

    // Threshold
    cv::Mat img_thresh;
    cv::threshold(img_undistorted, img_thresh, threshold_, 255, cv::THRESH_BINARY_INV);
    // PLOGI <<" Threshold";

    // Blob detection
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(blob_params_);
    blob_detector->detect(img_thresh, keypoints);
    // PLOGI <<" Blobs";

    if(output_debug)
    {
        std::string debug_path = id == CAMERA_ID::SIDE ? 
            cfg.lookup("vision_tracker.side.debug_output_path") :
            cfg.lookup("vision_tracker.rear.debug_output_path");
        cv::imwrite(debug_path + "img_raw.jpg", img_raw);
        cv::imwrite(debug_path + "img_undistorted.jpg", img_undistorted);
        cv::imwrite(debug_path + "img_thresh.jpg", img_thresh);
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img_thresh, keypoints, img_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite(debug_path + "img_keypoints.jpg", img_with_keypoints);
        cv::Mat img_with_best_keypoint = img_undistorted;
        cv::Point2f best_keypoint = getBestKeypoint(keypoints);
        cv::circle(img_with_best_keypoint, best_keypoint, 2, cv::Scalar(0,255,0), -1);
        cv::circle(img_with_best_keypoint, best_keypoint, 20, cv::Scalar(0,255,0), 5);
        Eigen::Vector2f best_point_m = cameraToRobot(best_keypoint, id);
        std::string label_text = std::to_string(best_point_m(0)) +" "+ std::to_string(best_point_m(1)) + " m";
        cv::putText(img_with_best_keypoint, //target image
                    label_text, //text
                    cv::Point(best_keypoint.x+20, best_keypoint.y+20), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.8,
                    CV_RGB(0,0,255), //font color
                    2);
        cv::imwrite(debug_path + "img_best_keypoint.jpg", img_with_best_keypoint);
    }

    return keypoints;
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

bool CameraTracker::isRunning()
{
    std::lock_guard<std::mutex> read_lock(running_mutex);
    return running_;
}

void CameraTracker::threadLoop()
{
    while(true)
    {
        if (isRunning())
        {
            oneLoop();
        }
    }
}

void CameraTracker::oneLoop()
{
    Eigen::Vector2f best_point_side = processImage(CAMERA_ID::SIDE);
    Eigen::Vector2f best_point_rear = processImage(CAMERA_ID::REAR);
    Point pose = computeRobotPoseFromImagePoints(best_point_side, best_point_rear);
    {
        std::lock_guard<std::mutex> read_lock(pose_mutex);
        current_point_ = pose;
    }
    {
        camera_loop_time_averager_.mark_point();
        std::lock_guard<std::mutex> read_lock(loop_time_mutex);
        loop_time_ms_ = camera_loop_time_averager_.get_ms();
    }
}

Eigen::Vector2f CameraTracker::processImage(CAMERA_ID id)
{
    cv::Mat frame;
    if (id == CAMERA_ID::REAR && use_debug_image_) 
    {
        frame = cameras_[CAMERA_ID::REAR].debug_frame;
    }
    else if (id == CAMERA_ID::REAR && !use_debug_image_) 
    {
        cameras_[CAMERA_ID::REAR].capture >> frame;
    }
    else if (id == CAMERA_ID::SIDE && use_debug_image_) 
    {
        frame = cameras_[CAMERA_ID::SIDE].debug_frame;
    }
    else if (id == CAMERA_ID::SIDE && !use_debug_image_) 
    {
        cameras_[CAMERA_ID::SIDE].capture >> frame;
    }
    else PLOGE << "Error: unknown camera id";

    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, id, output_debug_images_);
    cv::Point2f best_point_px = getBestKeypoint(keypoints);
    Eigen::Vector2f best_point_m = cameraToRobot(best_point_px, id);
    return best_point_m;
}

Eigen::Vector2f CameraTracker::cameraToRobot(cv::Point2f cameraPt, CAMERA_ID id)
{
    const CameraData& camera_data = cameras_[id];
    Eigen::Vector3f uv_point = {cameraPt.x, cameraPt.y, 1};

    // Compute scaling factor for Z=0
    Eigen::Vector3f tmp1 = camera_data.R_inv * camera_data.K_inv * uv_point;
    Eigen::Vector3f tmp2 = camera_data.R_inv * camera_data.t;
    float s = tmp2(2,0) / tmp1(2,0);

    // Convert to world coordinate
    Eigen::Vector3f world_point = camera_data.R_inv * (s * camera_data.K_inv * uv_point - camera_data.t);
    if(fabs(world_point(2)) > 1e-3) PLOGE.printf("Z value %f is not near zero", world_point(2));

    return {world_point(0),world_point(1)};
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
    float pose_x = -x[0];
    float pose_y = -x[1];
    float pose_a = -atan2(x[2], x[3]);
    return {pose_x, pose_y, pose_a};
}

