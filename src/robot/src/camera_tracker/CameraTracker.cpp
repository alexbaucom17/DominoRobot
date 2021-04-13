#include "CameraTracker.h"

#include <plog/Log.h>
#include <Eigen/Dense>
#include "constants.h"

CameraTracker::CameraTracker()
: camera_(0),
  running_(false)
{
    if (!camera_.isOpened()) {
        PLOGE << "ERROR: Could not open camera";
    }
    PLOGI << "Opened camera";

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

void CameraTracker::start()
{   
    running_ = true;
}

void CameraTracker::stop()
{
    running_ = false;

}

Point CameraTracker::getPoseFromCamera()
{
    return {0,0,0};
}

Eigen::Vector2f pointInImage(cv::Mat img_raw, int threshold, cv::SimpleBlobDetector::Params params, bool output_debug)
{

    // Convert to greyscale
    cv::Mat img_grey;
    cv::cvtColor(img_raw, img_grey, cv::COLOR_RGB2GRAY);

    // Threshold
    cv::Mat img_thresh;
    cv::threshold(img_grey, img_thresh, threshold, 255, cv::THRESH_BINARY);

    // Blob detection
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(params);
    blob_detector->detect(img_thresh, keypoints);

    for (const auto& k : keypoints) {
        PLOGI << "Keypoint: " << k.class_id;
        PLOGI << "Point: " << k.pt;
        PLOGI << "Angle: " << k.angle;
        PLOGI << "Size: " << k.size;
    }

    if(output_debug)
    {
        cv::imwrite("/home/pi/img_raw.jpg", img_raw);
        cv::imwrite("/home/pi/img_grey.jpg", img_grey);
        cv::imwrite("/home/pi/img_thresh.jpg", img_thresh);
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img_raw, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite("/home/pi/img_keypoints.jpg", img_with_keypoints);
    }

    return {0,0};
}

void CameraTracker::test_function()
{
    cv::Mat frame;
    camera_ >> frame;
    pointInImage(frame, threshold_, blob_params_, true);
    PLOGI << "Done with image processing";
}