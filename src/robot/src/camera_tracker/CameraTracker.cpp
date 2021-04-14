#include "CameraTracker.h"

#include <plog/Log.h>
#include "constants.h"

CameraTracker::CameraTracker()
: camera_(0),
  use_debug_image_(cfg.lookup("vision_tracker.use_debug_image")),
  current_point_({0,0,0}),
  pixels_per_meter_u_(cfg.lookup("vision_tracker.pixels_per_meter_u")),
  pixels_per_meter_v_(cfg.lookup("vision_tracker.pixels_per_meter_v"))
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
        current_frame_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if(current_frame_.empty())
        {
            PLOGE << "Could not read image from " << image_path;
            throw;
        }
        PLOGW << "Loading debug image from " << image_path;

    }

    // We are doing the thresholding manually
    threshold_ = cfg.lookup("vision_tracker.threshold");
    blob_params_.minThreshold = 10;
    blob_params_.maxThreshold = 200;

    blob_params_.filterByArea = cfg.lookup("vision_tracker.blob.use_area");
    blob_params_.minArea = cfg.lookup("vision_tracker.blob.min_area");
    blob_params_.maxArea = cfg.lookup("vision_tracker.blob.max_area");
    blob_params_.filterByColor = false;
    blob_params_.filterByCircularity = cfg.lookup("vision_tracker.blob.use_circularity");
    blob_params_.minCircularity = cfg.lookup("vision_tracker.blob.min_circularity");
    blob_params_.maxCircularity = cfg.lookup("vision_tracker.blob.max_circularity");
    blob_params_.filterByConvexity = false;
    blob_params_.filterByInertia = false;
}

CameraTracker::~CameraTracker() {}

cv::Point2f getBestKeypoint(std::vector<cv::KeyPoint> keypoints)
{
    if(keypoints.empty())
    {
        return {0,0};
    }
    
    // Get keypoint with largest area
    cv::KeyPoint best_keypoint = keypoints.front();
    for (const auto& k : keypoints) {
        PLOGI << "Keypoint: " << k.class_id;
        PLOGI << "Point: " << k.pt;
        PLOGI << "Angle: " << k.angle;
        PLOGI << "Size: " << k.size;
        if(k.size > best_keypoint.size)
        {
            best_keypoint = k;
        }
    }
    return best_keypoint.pt;
}

std::vector<cv::KeyPoint> CameraTracker::allKeypointsInImage(cv::Mat img_raw, bool output_debug)
{
    // Undistort and crop
    cv::Mat img_undistorted;
    cv::Rect validPixROI;
    cv::Mat newcameramtx = cv::getOptimalNewCameraMatrix(K_, D_, img_raw.size(), /*alpha=*/1, img_raw.size(), &validPixROI);
    cv::undistort(img_raw, img_undistorted, K_, D_);
    cv::Mat img_cropped = img_undistorted(validPixROI);

    // Threshold
    cv::Mat img_thresh;
    cv::threshold(img_cropped, img_thresh, threshold_, 255, cv::THRESH_BINARY_INV);
    // PLOGI <<" Threshold";

    // Blob detection
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(blob_params_);
    blob_detector->detect(img_thresh, keypoints);
    // PLOGI <<" Blobs";

    if(output_debug)
    {
        std::string debug_path = cfg.lookup("vision_tracker.debug_output_path");
        cv::imwrite(debug_path + "img_raw.jpg", img_raw);
        cv::imwrite(debug_path + "img_undistorted.jpg", img_undistorted);
        cv::imwrite(debug_path + "img_cropped.jpg", img_cropped);
        cv::imwrite(debug_path + "img_thresh.jpg", img_thresh);
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img_thresh, keypoints, img_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite(debug_path + "img_keypoints.jpg", img_with_keypoints);
        cv::Mat img_with_best_keypoint = img_cropped;
        cv::Point2f best_keypoint = getBestKeypoint(keypoints);
        cv::circle(img_with_best_keypoint, best_keypoint, 2, cv::Scalar(0,255,0), -1);
        cv::circle(img_with_best_keypoint, best_keypoint, 20, cv::Scalar(0,255,0), 5);
        cv::Point2f best_point_m = cameraToRobot(best_keypoint);
        std::string label_text = std::to_string(best_point_m.x) +" "+ std::to_string(best_point_m.y) + " m";
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
    if(!use_debug_image_) camera_ >> current_frame_;
    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(current_frame_, true);
    cv::Point2f best_point_px = getBestKeypoint(keypoints);
    cv::Point2f best_point_m = cameraToRobot(best_point_px);
    PLOGI << "Best keypoint at " << best_point_px << " px";
    PLOGI << "Best keypoint at " << best_point_m << " m";

    PLOGI << "Done with image processing";
}

cv::Point2f CameraTracker::cameraToRobot(cv::Point2f cameraPt)
{
    // TODO - use proper calibration for this
    float x =  (cameraPt.x - 320) / pixels_per_meter_u_;
    float y =  -1 * (cameraPt.y - 240) / pixels_per_meter_v_;
    return {x,y};
}

void CameraTracker::processImage()
{
    if(!use_debug_image_) camera_ >> current_frame_;
    std::vector<cv::KeyPoint> keypoints = allKeypointsInImage(current_frame_, false);
    cv::Point2f best_point_px = getBestKeypoint(keypoints);
    cv::Point2f best_point_m = cameraToRobot(best_point_px);
    PLOGI << "Best keypoint at " << best_point_px << " px";
    PLOGI << "Best keypoint at " << best_point_m << " m";
    current_point_ = {best_point_m.x, best_point_m.y, 0.0};
}