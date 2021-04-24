#ifndef CameraTracker_h
#define CameraTracker_h

#include "CameraTrackerBase.h"
#include "utils.h"
#include <Eigen/Dense>
#include "CameraPipeline.h"

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

    Point computeRobotPoseFromImagePoints(Eigen::Vector2f p_side, Eigen::Vector2f p_rear);

    void oneLoop();

  private:

    TimeRunningAverage camera_loop_time_averager_;
    int loop_time_ms_;
    CameraPipeline rear_cam_;
    CameraPipeline side_cam_;
    Eigen::Vector2f robot_P_side_target_;
    Eigen::Vector2f robot_P_rear_target_;
};


#endif //CameraTracker_h