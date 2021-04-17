#ifndef CameraTracker_h
#define CameraTracker_h

#include "CameraTrackerBase.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <map>

class CameraTracker : public CameraTrackerBase
{
  public:
    CameraTracker();
    
    virtual ~CameraTracker();

    virtual void start() override;

    virtual void stop() override;

    virtual Point getPoseFromCamera() override; 

    virtual int getLoopTimeMs() override;

    void test_function();   

  private:

    enum class CAMERA_ID
    {
      REAR,
      SIDE
    };

    std::string cameraIdToString(CAMERA_ID id);
    
    void initCamera(CAMERA_ID id);
    
    void threadLoop();

    bool isRunning();

    cv::Point2f processImage(CAMERA_ID id);

    std::vector<cv::KeyPoint> allKeypointsInImage(cv::Mat img_raw, CAMERA_ID id, bool output_debug);

    cv::Point2f cameraToRobot(cv::Point2f cameraPt);

    Point computeRobotPoseFromImagePoints(cv::Point2f p_side, cv::Point2f p_rear);

    struct CameraData 
    {
      CAMERA_ID id;
      cv::VideoCapture capture;
      cv::Mat K;
      cv::Mat D;
      cv::Mat debug_frame;
      std::string debug_output_path;
    };

    std::map<CAMERA_ID, CameraData> cameras_;
    bool use_debug_image_;
    bool output_debug_images_;
    bool running_;
    Point current_point_;
    TimeRunningAverage camera_loop_time_averager_;
    int loop_time_ms_;
    std::thread thread_;
    
    // Params
    cv::SimpleBlobDetector::Params blob_params_;
    int threshold_;
    float pixels_per_meter_u_;
    float pixels_per_meter_v_;
};


#endif //CameraTracker_h