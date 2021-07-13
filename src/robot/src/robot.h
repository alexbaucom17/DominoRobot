#ifndef Robot_h
#define Robot_h

#include "MarvelmindWrapper.h"
#include "RobotController.h"
#include "RobotServer.h"
#include "StatusUpdater.h"
#include "TrayController.h"
#include "utils.h"
#include "camera_tracker/CameraTracker.h"

class WaitForLocalizeHelper 
{
  public:
    WaitForLocalizeHelper(const StatusUpdater& statusUpdater, float max_timeout, float confidence_threshold);
    bool isDone();
    void start();

  private:
    const StatusUpdater& statusUpdater_;
    Timer timer_;
    float max_timeout_;
    float confidence_threshold_;
};


class Robot
{

  public:

    Robot();

    void run();
    void runOnce();

    // Used for tests only
    COMMAND getCurrentCommand() { return curCmd_; };
    StatusUpdater::Status getStatus() { return statusUpdater_.getStatus(); };

  private:

    bool checkForCmdComplete(COMMAND cmd);
    bool tryStartNewCmd(COMMAND cmd);
    bool checkForCameraStopTrigger();
    void resetCameraStopTriggers();

    StatusUpdater statusUpdater_;
    RobotServer server_;
    RobotController controller_;
    TrayController tray_controller_;
    MarvelmindWrapper mm_wrapper_;

    TimeRunningAverage position_time_averager_;    // Handles keeping average of the position update timing
    TimeRunningAverage robot_loop_time_averager_; 
    WaitForLocalizeHelper wait_for_localize_helper_;
    RateController vision_print_rate_;
    CameraTrackerBase* camera_tracker_;
    ClockTimePoint camera_motion_start_time_;
    ClockTimePoint camera_trigger_time_1_;
    ClockTimePoint camera_trigger_time_2_;
    bool camera_stop_triggered_;
    Point fine_move_target_;

    COMMAND curCmd_;
};


#endif