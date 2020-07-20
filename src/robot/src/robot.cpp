#include "RobotServer.h"
#include "RobotController.h"
#include "StatusUpdater.h"
#include "TrayController.h"
#include "utils.h"

#include <plog/Log.h> 
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ConsoleAppender.h>
#include <plog/Appenders/RollingFileAppender.h>

// Top level objects 
StatusUpdater statusUpdater;
RobotServer server = RobotServer(statusUpdater);
RobotController controller = RobotController(statusUpdater);
TrayController tray_controller = TrayController();

// TODO: Find new library for these
// RunningStatistics loop_time_averager;        // Handles keeping average of the loop timing
// RunningStatistics position_time_averager;    // Handles keeping average of the position update timing

// Variables used for loop
COMMAND newCmd = COMMAND::NONE;
COMMAND curCmd = COMMAND::NONE;
unsigned long prevLoopMillis = millis();
unsigned long prevPositionMillis = millis();


void configure_logger()
{
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender("Testlog.txt", 8000, 3); // Create the 1st appender.
    static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    // Step3: write log messages using a special macro
    // There are several log macros, use the macro you liked the most

    PLOG_INFO << "Logger ready";
}


void setup()
{
    configure_logger();

    #ifdef PRINT_DEBUG
    PLOGI.printf("Robot starting");
    #endif

    // Need this delay for controller to setup correctly for some reason
    usleep(100000);

    // Start server and controllers
    controller.begin();
    usleep(100000);
    tray_controller.begin();
    usleep(100000);
    server.begin();

    #ifdef PRINT_DEBUG
    PLOGI.printf("Done with setup, starting loop");
    #endif
}

bool tryStartNewCmd(COMMAND cmd)
{
    // Position info doesn't cound as a real 'command' since it doesn't interrupt anything
    // Always service it, but don't consider it starting a new command
    if (cmd == COMMAND::POSITION)
    {
        RobotServer::PositionData data = server.getPositionData();
        controller.inputPosition(data.x, data.y, data.a);

        // Update the position rate
        // position_time_averager.input(millis() - prevPositionMillis);
        prevPositionMillis = millis();

        return false;
    }
    // Same with ESTOP
    if (cmd == COMMAND::ESTOP)
    {
        controller.estop();
        tray_controller.estop();
        return false;
    }
    // Same with LOAD_COMPLETE
    if (cmd == COMMAND::LOAD_COMPLETE)
    {
        tray_controller.setLoadComplete();
        return false;
    }
    
    // For all other commands, we need to make sure we aren't doing anything else at the moment
    if(statusUpdater.getInProgress())
    {
        #ifdef PRINT_DEBUG
        PLOGI.printf("Command already running, rejecting new command");
        #endif
        return false;
    }
    
    // Start new command
    if(cmd == COMMAND::MOVE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPosition(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_REL)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionRelative(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_FINE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionFine(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_CONST_VEL)
    {
        RobotServer::VelocityData data = server.getVelocityData();
        controller.moveConstVel(data.vx, data.vy, data.va, data.t);
    }
    else if(cmd == COMMAND::PLACE_TRAY)
    {
        tray_controller.place();
    }
    else if(cmd == COMMAND::LOAD_TRAY)
    {
        tray_controller.load();
    }
    else if(cmd == COMMAND::INITIALIZE_TRAY)
    {
        tray_controller.initialize();
    }
    else if (cmd == COMMAND::NONE)
    {
      // do nothing...
    }
    else
    {
        #ifdef PRINT_DEBUG
        PLOGI.printf("Unknown command!");
        #endif
        return false;
    }

    return true;
}

bool checkForCmdComplete(COMMAND cmd)
{
    if (cmd == COMMAND::NONE)
    {
        return true;
    }
    else if(cmd == COMMAND::MOVE || 
            cmd == COMMAND::MOVE_REL ||
            cmd == COMMAND::MOVE_FINE ||
            cmd == COMMAND::MOVE_CONST_VEL)
    {
        return controller.isTrajectoryRunning();
    }
    else if(cmd == COMMAND::PLACE_TRAY ||
            cmd == COMMAND::LOAD_TRAY ||
            cmd == COMMAND::INITIALIZE_TRAY)
    {
        return tray_controller.isActionRunning();
    }
    else
    {
        #ifdef PRINT_DEBUG
        PLOGI.printf("Completion check not implimented for command: %i",cmd);
        #endif
        return true;
    }
    
}

void loop() 
{
    // Check for new command and try to start it
    newCmd = server.oneLoop();
    bool status = tryStartNewCmd(newCmd);

    // Update our current command if we successfully started a new command
    if(status)
    {
        curCmd = newCmd;
        statusUpdater.updateInProgress(true);
    }

    // Service controllers
    controller.update();
    tray_controller.update();

    // Check if the current command has finished
    bool done = checkForCmdComplete(curCmd);
    if(done)
    {
        curCmd = COMMAND::NONE;
        statusUpdater.updateInProgress(false);
    }

    // Update loop time and status updater
    // loop_time_averager.input(static_cast<float>(millis() - prevLoopMillis));
    prevLoopMillis = millis();
    // statusUpdater.updateLoopTimes(static_cast<int>(loop_time_averager.mean()), static_cast<int>(position_time_averager.mean()));
    
}


// For simpler porting
// TODO: Cleanup
int main()
{
    setup();

    while(true)
    {
        loop();
    }
    return 0;
}
