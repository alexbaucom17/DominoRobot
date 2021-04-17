#ifndef CameraTracker_h
#define CameraTracker_h

#include "CameraTrackerBase.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <thread>
#include <map>

class CameraTracker : public CameraTrackerBase
{
  public:
    CameraTracker(bool start_thread = true);
    
    virtual ~CameraTracker();

    virtual void start() override;

    virtual void stop() override;

    virtual Point getPoseFromCamera() override; 

    virtual int getLoopTimeMs() override;

    void test_function();   

    // Below here exposed for testing only (yes, bad practice, but useful for now)
    enum class CAMERA_ID
    {
      REAR,
      SIDE
    };

    Eigen::Vector2f processImage(CAMERA_ID id);

    std::vector<cv::KeyPoint> allKeypointsInImage(cv::Mat img_raw, CAMERA_ID id, bool output_debug);

    Eigen::Vector2f cameraToRobot(cv::Point2f cameraPt, CAMERA_ID id);

    Eigen::Vector2f robotToCamera(Eigen::Vector2f robotPt, CAMERA_ID id);

    Point computeRobotPoseFromImagePoints(Eigen::Vector2f p_side, Eigen::Vector2f p_rear);

    void oneLoop();

  private:

    std::string cameraIdToString(CAMERA_ID id);
    
    void initCamera(CAMERA_ID id);
    
    void threadLoop();

    bool isRunning();

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

    std::map<CAMERA_ID, CameraData> cameras_;
    bool use_debug_image_;
    bool output_debug_images_;
    bool running_;
    bool thread_running_;
    Point current_point_;
    TimeRunningAverage camera_loop_time_averager_;
    int loop_time_ms_;
    std::thread thread_;
    
    // Params
    cv::SimpleBlobDetector::Params blob_params_;
    int threshold_;
    float pixels_per_meter_u_;
    float pixels_per_meter_v_;
    Eigen::Vector2f robot_P_side_target_;
    Eigen::Vector2f robot_P_rear_target_;
};


#endif //CameraTracker_h