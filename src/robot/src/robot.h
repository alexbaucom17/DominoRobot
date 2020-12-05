#ifndef Robot_h
#define Robot_h

#include "MarvelmindWrapper.h"
#include "RobotController.h"
#include "RobotServer.h"
#include "StatusUpdater.h"
#include "TrayController.h"
#include "utils.h"

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

    StatusUpdater statusUpdater_;
    RobotServer server_;
    RobotController controller_;
    TrayController tray_controller_;
    MarvelmindWrapper mm_wrapper_;

    TimeRunningAverage loop_time_averager_;        // Handles keeping average of the loop timing
    TimeRunningAverage position_time_averager_;    // Handles keeping average of the position update timing

    COMMAND curCmd_;
};


#endif