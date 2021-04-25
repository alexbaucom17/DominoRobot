#include "CameraPipeline.h"

#include <plog/Log.h>
#include "constants.h"
#include <mutex>

std::mutex data_mutex;

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

CameraPipeline::CameraPipeline(CAMERA_ID id, bool start_thread)
: use_debug_image_(cfg.lookup("vision_tracker.debug.use_debug_image")),
  output_debug_images_(cfg.lookup("vision_tracker.debug.save_camera_debug")),
  thread_running_(false),
  current_output_()
{
    initCamera(id);

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
    blob_detector_ = cv::SimpleBlobDetector::create(blob_params_);

    // Start thread
    if (start_thread) start();
}

CameraPipeline::~CameraPipeline()
{
    stop();
}

void CameraPipeline::initCamera(CAMERA_ID id)
{
    // Get config strings
    std::string name = cameraIdToString(id);
    std::string calibration_path_config_name = "vision_tracker." + name + ".calibration_file";
    std::string camera_path_config_name = "vision_tracker." + name + ".camera_path";
    std::string debug_output_path_config_name = "vision_tracker." + name + ".debug_output_path";
    std::string debug_image_config_name = "vision_tracker." + name + ".debug_image";
    std::string x_offset_config_name = "vision_tracker.physical." + name + ".x_offset";
    std::string y_offset_config_name = "vision_tracker.physical." + name + ".y_offset";
    std::string z_offset_config_name = "vision_tracker.physical." + name + ".z_offset";
    std::string x_res_scale_config_name = "vision_tracker." + name + ".resolution_scale_x";
    std::string y_res_scale_config_name = "vision_tracker." + name + ".resolution_scale_y";
    
    // Initialize intrinsic calibration data
    std::string calibration_path = cfg.lookup(calibration_path_config_name);
    PLOGI.printf("Loading %s camera calibration from %s", name.c_str(), calibration_path.c_str());
    cv::FileStorage fs(calibration_path, cv::FileStorage::READ);
    if(fs["K"].isNone() || fs["D"].isNone()) 
    {
        PLOGE.printf("Missing %s calibration data", name.c_str());
        throw;
    }
    fs["K"] >> camera_data_.K;
    fs["D"] >> camera_data_.D;

    // Handle scale factors
    float x_res_scale = cfg.lookup(x_res_scale_config_name);
    camera_data_.K.at<double>(0,0) = camera_data_.K.at<double>(0,0) * x_res_scale;
    camera_data_.K.at<double>(0,1) = camera_data_.K.at<double>(0,1) * x_res_scale;
    camera_data_.K.at<double>(0,2) = camera_data_.K.at<double>(0,2) * x_res_scale;
    float y_res_scale = cfg.lookup(y_res_scale_config_name);
    camera_data_.K.at<double>(1,0) = camera_data_.K.at<double>(1,0) * y_res_scale;
    camera_data_.K.at<double>(1,1) = camera_data_.K.at<double>(1,1) * y_res_scale;
    camera_data_.K.at<double>(1,2) = camera_data_.K.at<double>(1,2) * y_res_scale;

    // Initialize extrinsic data
    float x_offset = cfg.lookup(x_offset_config_name);
    float y_offset = cfg.lookup(y_offset_config_name);
    float z_offset = cfg.lookup(z_offset_config_name);
    Eigen::Vector3f camera_pose = {x_offset, y_offset, z_offset};
    Eigen::Matrix3f camera_rotation;
    // Hardcoding rotation matrices here for simplicity
    if(id == CAMERA_ID::SIDE) 
    {
        camera_rotation << 0,1,0, 1,0,0, 0,0,-1;
    }
    else
    {
        camera_rotation << 1,0,0, 0,-1,0, 0,0,-1;
    }
    // From https://ksimek.github.io/2012/08/22/extrinsic/
    camera_data_.t = -1 * camera_rotation * camera_pose;
    camera_data_.R =  camera_rotation.transpose();
    // PLOGI << name << " R: " << camera_data_.R;
    // PLOGI << name << " t: " << camera_data_.t.transpose();

    // Precompute some inverse values for later re-use
    camera_data_.R_inv = camera_data_.R.inverse();
    Eigen::Matrix3f tmp;
    // Hack to get around problems with enabling eigen support in opencv
    tmp << camera_data_.K.at<double>(0,0), camera_data_.K.at<double>(0,1), camera_data_.K.at<double>(0,2),
           camera_data_.K.at<double>(1,0), camera_data_.K.at<double>(1,1), camera_data_.K.at<double>(1,2),
           camera_data_.K.at<double>(2,0), camera_data_.K.at<double>(2,1), camera_data_.K.at<double>(2,2);
    camera_data_.K_inv = tmp.inverse();

    // PLOGI << name << " K: " << camera_data_.K;
    // PLOGI << name << " Ktmp: " << tmp;
    // PLOGI << name << " Kinv: " << camera_data_.K_inv;
    
    // Initialize camera calibration capture or debug data
    if(!use_debug_image_)
    {
        std::string camera_path = cfg.lookup(camera_path_config_name);
        camera_data_.capture = cv::VideoCapture(camera_path);
        if (!camera_data_.capture.isOpened()) 
        {
            PLOGE.printf("Could not open %s camera at %s", name.c_str(), camera_path.c_str());
            throw;
        }
        PLOGI.printf("Opened %s camera at %s", name.c_str(), camera_path.c_str());
    } 
    else 
    {
        std::string image_path = cfg.lookup(debug_image_config_name);
        camera_data_.debug_frame = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if(camera_data_.debug_frame.empty())
        {
            PLOGE.printf("Could not read %s debug image from %s", name.c_str(), image_path.c_str());
            throw;
        }
        PLOGW.printf("Loading %s debug image from %s", name.c_str(), image_path.c_str());
    }

    // Initialze debug output path
    if(output_debug_images_)
    {
        camera_data_.debug_output_path = std::string(cfg.lookup(debug_output_path_config_name)); 
    }
}

std::string CameraPipeline::cameraIdToString(CAMERA_ID id)
{
    if(id == CAMERA_ID::SIDE) return "side";
    if(id == CAMERA_ID::REAR) return "rear";
    PLOGE << "Unknown camera id";
    return "unk";
}

std::vector<cv::KeyPoint> CameraPipeline::allKeypointsInImage(cv::Mat img_raw, bool output_debug)
{   
    // Timer t;

    // Undistort and crop
    cv::Mat img_undistorted;
    cv::Rect validPixROI;
    // PLOGI << "init memory time: " << t.dt_ms();
    // t.reset();
    cv::Mat newcameramtx = cv::getOptimalNewCameraMatrix(camera_data_.K, camera_data_.D, img_raw.size(), /*alpha=*/1, img_raw.size(), &validPixROI);
    // PLOGI << "new camera time: " << t.dt_ms();
    // t.reset();
    cv::undistort(img_raw, img_undistorted, camera_data_.K, camera_data_.D);

    // PLOGI << "undistort time: " << t.dt_ms();
    // t.reset();

    // Threshold
    cv::Mat img_thresh;
    cv::threshold(img_undistorted, img_thresh, threshold_, 255, cv::THRESH_BINARY_INV);
    // PLOGI <<" Threshold";

    // PLOGI << "threshold time: " << t.dt_ms();
    // t.reset();

    // Blob detection
    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(10);
    // PLOGI << "blob memory time: " << t.dt_ms();
    // t.reset();
    blob_detector_->detect(img_thresh, keypoints);
    // PLOGI <<"Num blobs " << keypoints.size();

    // PLOGI << "blob detect time: " << t.dt_ms();
    // t.reset();

    if(output_debug)
    {
        std::string debug_path = camera_data_.id == CAMERA_ID::SIDE ? 
            cfg.lookup("vision_tracker.side.debug_output_path") :
            cfg.lookup("vision_tracker.rear.debug_output_path");
        cv::imwrite(debug_path + "img_raw.jpg", img_raw);
        cv::imwrite(debug_path + "img_undistorted.jpg", img_undistorted);
        cv::imwrite(debug_path + "img_thresh.jpg", img_thresh);

        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img_thresh, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite(debug_path + "img_keypoints.jpg", img_with_keypoints);

        cv::Mat img_with_best_keypoint = img_undistorted;
        cv::Point2f best_keypoint = getBestKeypoint(keypoints);
        cv::circle(img_with_best_keypoint, best_keypoint, 2, cv::Scalar(0,0,255), -1);
        cv::circle(img_with_best_keypoint, best_keypoint, 20, cv::Scalar(0,0,255), 5);
        Eigen::Vector2f best_point_m = cameraToRobot(best_keypoint);
        std::string label_text = "Best point: " + std::to_string(best_point_m(0)) +"m, "+ std::to_string(best_point_m(1)) + " m";
        cv::putText(img_with_best_keypoint, //target image
                    label_text, //text
                    cv::Point(20, 20), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.8,
                    CV_RGB(255,0,0), //font color
                    2);
        Eigen::Vector2f target_point_world;
        if(camera_data_.id == CAMERA_ID::SIDE)
        {
            target_point_world << float(cfg.lookup("vision_tracker.physical.side.target_x")), float(cfg.lookup("vision_tracker.physical.side.target_y"));
        }
        else 
        {
            target_point_world << float(cfg.lookup("vision_tracker.physical.rear.target_x")), float(cfg.lookup("vision_tracker.physical.rear.target_y"));
        }
        Eigen::Vector2f target_point_camera = robotToCamera(target_point_world);
        cv::Point2f pt{target_point_camera[0], target_point_camera[1]};
        cv::circle(img_with_best_keypoint, pt, 5, cv::Scalar(255,0,0), -1);
        std::string label_text2 = "Target point: " + std::to_string(target_point_camera[0]) +"px, "+ std::to_string(target_point_camera[1]) + " px";
        cv::putText(img_with_best_keypoint, //target image
                    label_text2, //text
                    cv::Point(20, 60), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.8,
                    CV_RGB(0,0,255), //font color
                    2);

        cv::imwrite(debug_path + "img_best_keypoint.jpg", img_with_best_keypoint);
        PLOGI << "Writing debug images";
    }

    // PLOGI << "debug time: " << t.dt_ms();
    // t.reset();

    return keypoints;
}

void CameraPipeline::start()
{
    if(!thread_running_)
    {
        PLOGI.printf("Starting %s camera thread",cameraIdToString(camera_data_.id).c_str());
        thread_running_ = true;
        thread_ = std::thread(&CameraPipeline::threadLoop, this);
        thread_.detach();
    }
}

void CameraPipeline::stop()
{
    if(thread_running_)
    {
        PLOGI.printf("Stopping %s camera thread",cameraIdToString(camera_data_.id).c_str());
        thread_running_ = false;
    }
}

void CameraPipeline::threadLoop()
{
    while(thread_running_)
    {
        oneLoop();
        PLOGI << "loop";
    }
}

void CameraPipeline::oneLoop()
{
    // Timer t;
    Eigen::Vector2f best_point = processImage();
    // PLOGI << cameraIdToString(camera_data_.id) <<" cam time: " << t.dt_ms();
    // t.reset();
    {
        std::lock_guard<std::mutex> read_lock(data_mutex);
        current_output_.ok = true;
        current_output_.timestamp = ClockFactory::getFactoryInstance()->get_clock()->now();
        current_output_.point = best_point;
    }
//     PLOGI << "finish time: " << t.dt_ms();
//     t.reset();
}

CameraPipelineOutput CameraPipeline::getData()
{
    std::lock_guard<std::mutex> read_lock(data_mutex);
    CameraPipelineOutput output = current_output_;
    current_output_.ok = false;
    return output;
}

Eigen::Vector2f CameraPipeline::processImage()
{
    // Timer t;
    cv::Mat frame;
    if (use_debug_image_) 
    {
        frame = camera_data_.debug_frame;
    }
    else
    {
        camera_data_.capture >> frame;
    }

    // PLOGI << "frame time: " << t.dt_ms();
    // t.reset();

    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(frame, output_debug_images_);
    // PLOGI << "detection time: " << t.dt_ms();
    // t.reset();
    cv::Point2f best_point_px = getBestKeypoint(keypoints);
    // PLOGI << "filter time: " << t.dt_ms();
    // t.reset();
    Eigen::Vector2f best_point_m = cameraToRobot(best_point_px);
    // PLOGI << "transform time: " << t.dt_ms();
    // t.reset();
    return best_point_m;
}

Eigen::Vector2f CameraPipeline::cameraToRobot(cv::Point2f cameraPt)
{
    Eigen::Vector3f uv_point = {cameraPt.x, cameraPt.y, 1};

    // Compute scaling factor for Z=0
    Eigen::Vector3f tmp1 = camera_data_.R_inv * camera_data_.K_inv * uv_point;
    Eigen::Vector3f tmp2 = camera_data_.R_inv * camera_data_.t;
    float s = tmp2(2,0) / tmp1(2,0);

    // Debug
    Eigen::Vector3f scaled_cam_frame = s * camera_data_.K_inv * uv_point;
    Eigen::Vector3f cam_frame_offset = scaled_cam_frame  - camera_data_.t;

    // Convert to world coordinate
    Eigen::Vector3f world_point = camera_data_.R_inv * cam_frame_offset;
    // PLOGI << cameraIdToString(id);
    // PLOGI << "uv_point: " << uv_point.transpose();
    // PLOGI << "scaled_cam_frame: " << scaled_cam_frame.transpose();
    // PLOGI << "cam_frame_offset: " << cam_frame_offset.transpose();
    // PLOGI << "world_point: " << world_point.transpose();


    if(fabs(world_point(2)) > 1e-3) PLOGE.printf("Z value %f is not near zero", world_point(2));

    return {world_point(0),world_point(1)};
}

Eigen::Vector2f CameraPipeline::robotToCamera(Eigen::Vector2f robotPt)
{
    Eigen::Vector4f robot_point_3d = {robotPt[0], robotPt[1], 0, 1};
    Eigen::MatrixXf r_t(3,4);
    r_t << camera_data_.R, camera_data_.t;

    Eigen::Matrix3f K_tmp;
    // Hack to get around problems with enabling eigen support in opencv
    K_tmp << camera_data_.K.at<double>(0,0), camera_data_.K.at<double>(0,1), camera_data_.K.at<double>(0,2),
           camera_data_.K.at<double>(1,0), camera_data_.K.at<double>(1,1), camera_data_.K.at<double>(1,2),
           camera_data_.K.at<double>(2,0), camera_data_.K.at<double>(2,1), camera_data_.K.at<double>(2,2);

    Eigen::Vector3f camera_pt_scaled = K_tmp * r_t * robot_point_3d;
    return {camera_pt_scaled[0]/camera_pt_scaled[2], camera_pt_scaled[1]/camera_pt_scaled[2]};
}