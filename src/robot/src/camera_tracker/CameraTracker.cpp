#include "CameraTracker.h"

#include <plog/Log.h>

CameraTracker::CameraTracker()
: running_(false)
{
    cv::SimpleBlobDetector::Params params;
    
    params.filterByColor = true;
    params.blobColor = 0;
    
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 50;
    params.maxArea = 100000;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.5;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    blob_detector_ = cv::SimpleBlobDetector::create(params);
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

void CameraTracker::test_function()
{
    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        PLOGE << "ERROR: Could not open camera";
        return;
    }
    PLOGI << "Opened camera";
    cv::Mat frame;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat img_with_keypoints;
    PLOGI << "Allocated memory";

    TimeRunningAverage averager(10);

    for (int i = 0; i < 50; i++)
    {
        camera >> frame;
        // PLOGI << "Got frame";
        
        // cv::imwrite("/home/pi/debug_img.jpg", frame);
        blob_detector_->detect(frame, keypoints);
        // PLOGI << "Got keypoints";

        // cv::drawKeypoints(frame, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // cv::imwrite("/home/pi/debug_img2.jpg", img_with_keypoints);
        // PLOGI << "Wrote test images";
        averager.mark_point();
        
        if (i%10 == 0)
        {
            PLOGI<< "Average ms for cycle: "<< averager.get_ms();
        }
    }
}