#include "CameraTracker.h"

#include <plog/Log.h>
#include "constants.h"

CameraTracker::CameraTracker()
: camera_(0),
  use_debug_image_(cfg.lookup("vision_tracker.use_debug_image")),
  current_point_({0,0,0})
{
    // Load camera calibration parameters
    std::string calibration_path = cfg.lookup("vision_tracker.calibration");
    PLOGI << "Loading camera calibration from " << calibration_path;
    cv::FileStorage fs(calibration_path, cv::FileStorage::READ);
    if(fs["K"].isNone() || fs["D"].isNone()) 
    {
        PLOGE << "ERROR: Missing calibration data";
        throw;
    }
    fs["K"] >> K_;
    fs["D"] >> D_;

    if(!use_debug_image_)
    {
        if (!camera_.isOpened()) 
        {
            PLOGE << "ERROR: Could not open camera";
            throw;
        }
        PLOGI << "Opened camera";
    }
    else
    {
        std::string image_path = cfg.lookup("vision_tracker.debug_image");
        current_frame_ = cv::imread(image_path);
        if(current_frame_.empty())
        {
            PLOGE << "Could not read image from " << image_path;
            throw;
        }
        PLOGW << "Loading debug image from " << image_path;

    }

    // We are doing the thresholding manually
    threshold_ = cfg.lookup("vision_tracker.threshold");
    blob_params_.minThreshold = 0;
    blob_params_.maxThreshold = 255;

    blob_params_.filterByArea = cfg.lookup("vision_tracker.blob.use_area");
    blob_params_.minArea = cfg.lookup("vision_tracker.blob.min_area");;
    blob_params_.maxArea = cfg.lookup("vision_tracker.blob.max_area");;
    blob_params_.filterByColor = false;
    blob_params_.filterByCircularity = false;
    blob_params_.filterByConvexity = false;
    blob_params_.filterByInertia = false;
}

CameraTracker::~CameraTracker() {}

std::vector<cv::KeyPoint> CameraTracker::allKeypointsInImage(cv::Mat img_raw, bool output_debug)
{
    // Undistort and crop
    cv::Mat img_undistorted;
    cv::Rect validPixROI;
    cv::Mat newcameramtx = cv::getOptimalNewCameraMatrix(K_, D_, img_raw.size(), /*alpha=*/1, img_raw.size(), &validPixROI);
    cv::undistort(img_raw, img_undistorted, K_, D_);
    cv::Mat cropped_image = img_undistorted(validPixROI);

    // Convert to greyscale
    cv::Mat img_grey;
    cv::cvtColor(cropped_image, img_grey, cv::COLOR_RGB2GRAY);

    // Threshold
    cv::Mat img_thresh;
    cv::threshold(img_grey, img_thresh, threshold_, 255, cv::THRESH_BINARY);

    // Blob detection
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(blob_params_);
    blob_detector->detect(img_thresh, keypoints);

    if(output_debug)
    {
        std::string debug_path = cfg.lookup("vision_tracker.debug_output_path");
        cv::imwrite(debug_path + "img_raw.jpg", img_raw);
        cv::imwrite(debug_path + "img_undistorted.jpg", img_undistorted);
        cv::imwrite(debug_path + "img_grey.jpg", img_grey);
        cv::imwrite(debug_path + "img_thresh.jpg", img_thresh);
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img_grey, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite(debug_path + "img_keypoints.jpg", img_with_keypoints);
    }

    return keypoints;
}

cv::Point2f getLargestKeypoint(std::vector<cv::KeyPoint> keypoints)
{
    // Get keypoint with largest area
    cv::KeyPoint best_keypoint = keypoints[0];
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

void CameraTracker::test_function()
{
    if(!use_debug_image_) camera_ >> current_frame_;
    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(current_frame_, true);
    cv::Point2f best_point = getLargestKeypoint(keypoints);
    PLOGI << "Best keypoint at " << best_point;

    PLOGI << "Done with image processing";
}

void CameraTracker::processImage()
{
    if(!use_debug_image_) camera_ >> current_frame_;
    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(current_frame_, false);
    cv::Point2f best_point = getLargestKeypoint(keypoints);
}