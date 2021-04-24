#ifndef CameraPipeline_h
#define CameraPipeline_h

#include "utils.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <thread>

enum class CAMERA_ID
{
    REAR,
    SIDE
};

class CameraPipeline 
{
  public:

    CameraPipeline(CAMERA_ID id, bool start_thread);

    bool isDataReady();

    CameraPipelineOutput getData();
  
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

    threadLoop();

    std::string cameraIdToString(CAMERA_ID id);
    
    void initCamera(CAMERA_ID id);

    Eigen::Vector2f processImage();

    std::vector<cv::KeyPoint> allKeypointsInImage(cv::Mat img_raw, bool output_debug);

    Eigen::Vector2f cameraToRobot(cv::Point2f cameraPt);

    Eigen::Vector2f robotToCamera(Eigen::Vector2f robotPt);

    CameraData camera_data_;
    bool use_debug_image_;
    bool output_debug_images_;
    bool thread_running_;
    std::thread thread_;
    Eigen::Vector2f current_point_;

    // Params read from config
    cv::SimpleBlobDetector::Params blob_params_;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector_;
    int threshold_;
    float pixels_per_meter_u_;
    float pixels_per_meter_v_;


};

#endif //CameraPipeline_h