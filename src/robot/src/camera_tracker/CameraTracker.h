#ifndef CameraTracker_h
#define CameraTracker_h

#include "CameraTrackerBase.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

class CameraTracker : public CameraTrackerBase
{
  public:
    CameraTracker();

    virtual ~CameraTracker();

    virtual void processImage() override;

    virtual Point getPoseFromCamera() override {return current_point_;}; 

    void test_function();   

  private:

    std::vector<cv::KeyPoint> allKeypointsInImage(cv::Mat img_raw, bool output_debug);

    cv::Point2f cameraToRobot(cv::Point2f cameraPt);

    cv::VideoCapture side_camera_;
    cv::VideoCapture rear_camera_;
    cv::SimpleBlobDetector::Params blob_params_;
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat current_frame_;
    int threshold_;
    bool use_debug_image_;
    bool running_;
    Point current_point_;
    float pixels_per_meter_u_;
    float pixels_per_meter_v_;
    
};


#endif //CameraTracker_h