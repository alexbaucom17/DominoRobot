#include "CameraTracker.h"

#include <plog/Log.h>

CameraTracker::CameraTracker()
: blob_detector_(),
  running_(false)
{

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
    cv::Mat frame;
    camera >> frame;
    cv::imwrite("/home/pi/test.jpg", frame);
    PLOGI << "Wrote test image";
}