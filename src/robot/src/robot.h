#ifndef Robot_h
#define Robot_h

#include "RobotServer.h"
#include "RobotController.h"
#include "StatusUpdater.h"
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
    // TrayController tray_controller = TrayController();

    
    // Variables used for loop
    COMMAND newCmd;
    COMMAND curCmd;

    // TODO: Find new library for these
    // RunningStatistics loop_time_averager;        // Handles keeping average of the loop timing
    // RunningStatistics position_time_averager;    // Handles keeping average of the position update timing
};


#endif