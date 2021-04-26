#ifndef CameraPipeline_h
#define CameraPipeline_h

#include "utils.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <thread>
#include <atomic>

enum class CAMERA_ID
{
    REAR,
    SIDE
};

struct CameraPipelineOutput
{
  bool ok = false;
  ClockTimePoint timestamp = ClockFactory::getFactoryInstance()->get_clock()->now();
  Eigen::Vector2f point = {0,0};
  Eigen::Vector2f uv = {0,0};
};

class CameraPipeline 
{
  public:

    CameraPipeline(CAMERA_ID id, bool start_thread);

    ~CameraPipeline();

    CameraPipelineOutput getData();

    void start();

    void stop();

    void oneLoop();

    Eigen::Vector2f cameraToRobot(cv::Point2f cameraPt);

    Eigen::Vector2f robotToCamera(Eigen::Vector2f robotPt);
  
  private:

    struct CameraData 
    {
      CAMERA_ID id;
      cv::VideoCapture capture;
      cv::Mat K;
      Eigen::Matrix3f K_inv;
      cv::Mat D;
      Eigen::Matrix3f R;
      Eigen::Matrix3f R_inv;
      Eigen::Vector3f t;
      cv::Mat debug_frame;
      std::string debug_output_path;
    };

    void threadLoop();

    std::string cameraIdToString(CAMERA_ID id);
    
    void initCamera(CAMERA_ID id);

    std::vector<cv::KeyPoint> allKeypointsInImage(cv::Mat img_raw, bool output_debug);

    CameraData camera_data_;
    bool use_debug_image_;
    bool output_debug_images_;
    std::atomic<bool> thread_running_;
    std::thread thread_;
    CameraPipelineOutput current_output_;

    // Params read from config
    cv::SimpleBlobDetector::Params blob_params_;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector_;
    int threshold_;
    float pixels_per_meter_u_;
    float pixels_per_meter_v_;
};

#endif //CameraPipeline_h