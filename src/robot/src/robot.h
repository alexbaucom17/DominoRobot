#ifndef Robot_h
#define Robot_h

#include "MarvelmindWrapper.h"
#include "RobotController.h"
#include "RobotServer.h"
#include "StatusUpdater.h"
#include "TimeRunningAverage.h"
#include "TrayController.h"
#include "utils.h"

class Robot
{

  public:

    Robot();

    void run();

  private:

    bool checkForCmdComplete(COMMAND cmd);
    bool tryStartNewCmd(COMMAND cmd);


    StatusUpdater statusUpdater;
    RobotServer server;
    RobotController controller;
    TrayController tray_controller;
    MarvelmindWrapper mm_wrapper;

    TimeRunningAverage loop_time_averager;        // Handles keeping average of the loop timing
    TimeRunningAverage position_time_averager;    // Handles keeping average of the position update timing
};


#endif