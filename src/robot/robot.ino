/* Requires libraries:
*  Encoder: https://github.com/PaulStoffregen/Encoder
*  Filters: https://github.com/JonHub/Filters
*  PID: https://playground.arduino.cc/Code/PIDLibrary/
*  ArduinoJson: https://arduinojson.org
*  LinAlgebra: https://github.com/dams666/LinAlgebra
*   (modified to reduce matrix size to 3)
*  ArduinoSTL: https://www.arduinolibraries.info/libraries/arduino-stl
*  MemoryFree: https://playground.arduino.cc/Code/AvailableMemory/
*  AccelStepper: https://www.airspayce.com/mikem/arduino/AccelStepper/
*/

#include "RobotServer.h"
#include "RobotController.h"
#include "StatusUpdater.h"
#include "TrayController.h"
#include <Filters.h>

// Top level objects 
StatusUpdater statusUpdater;
RobotServer server = RobotServer(Serial3, Serial, statusUpdater);
RobotController controller = RobotController(Serial, statusUpdater);
TrayController tray_controller = TrayController(Serial);
RunningStatistics loop_time_averager;        // Handles keeping average of the loop timing
RunningStatistics position_time_averager;    // Handles keeping average of the position update timing

// Variables used for loop
RobotServer::COMMAND newCmd = RobotServer::COMMAND::NONE;
RobotServer::COMMAND curCmd = RobotServer::COMMAND::NONE;
unsigned long prevLoopMillis = millis();
unsigned long prevPositionMillis = millis();


void setup()
{
    // Communication with the host computer
    Serial.begin(115200); 
    #ifdef PRINT_DEBUG
    Serial.println("Robot starting");
    #endif

    // Need this delay for controller to setup correctly for some reason
    delay(100);

    // Start server and controllers
    controller.begin();
    delay(100);
    tray_controller.begin();
    delay(100);
    server.begin();

    #ifdef PRINT_DEBUG
    Serial.println("Done with setup, starting loop");
    #endif
}

bool tryStartNewCmd(RobotServer::COMMAND cmd)
{
    // Position info doesn't cound as a real 'command' since it doesn't interrupt anything
    // Always service it, but don't consider it starting a new command
    if (cmd == RobotServer::COMMAND::POSITION)
    {
        RobotServer::PositionData data = server.getPositionData();
        controller.inputPosition(data.x, data.y, data.a);

        // Update the position rate
        position_time_averager.input(millis() - prevPositionMillis);
        prevPositionMillis = millis();

        return false;
    }
    // Same with ESTOP
    if (cmd == RobotServer::COMMAND::ESTOP)
    {
        controller.estop();
        tray_controller.estop();
        return false;
    }
    // Same with LOAD_COMPLETE
    if (cmd == RobotServer::COMMAND::LOAD_COMPLETE)
    {
        tray_controller.setLoadComplete();
        return false;
    }
    
    // For all other commands, we need to make sure we aren't doing anything else at the moment
    if(statusUpdater.getInProgress())
    {
        #ifdef PRINT_DEBUG
        Serial.println("Command already running, rejecting new command");
        #endif
        return false;
    }
    
    // Start new command
    if(cmd == RobotServer::COMMAND::MOVE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPosition(data.x, data.y, data.a);
    }
    else if(cmd == RobotServer::COMMAND::MOVE_REL)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionRelative(data.x, data.y, data.a);
    }
    else if(cmd == RobotServer::COMMAND::MOVE_FINE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionFine(data.x, data.y, data.a);
    }
    else if(cmd == RobotServer::COMMAND::PLACE_TRAY)
    {
        tray_controller.place();
    }
    else if(cmd == RobotServer::COMMAND::LOAD_TRAY)
    {
        tray_controller.load();
    }
    else if(cmd == RobotServer::COMMAND::INITIALIZE_TRAY)
    {
        tray_controller.initialize();
    }
    else if (cmd == RobotServer::COMMAND::NONE)
    {
      // do nothing...
    }
    else
    {
        #ifdef PRINT_DEBUG
        Serial.println("Unknown command!");
        #endif
        return false;
    }

    return true;
}

bool checkForCmdComplete(RobotServer::COMMAND cmd)
{
    if (cmd == RobotServer::COMMAND::NONE)
    {
        return true;
    }
    else if(cmd == RobotServer::COMMAND::MOVE || 
            cmd == RobotServer::COMMAND::MOVE_REL ||
            cmd == RobotServer::COMMAND::MOVE_FINE)
    {
        return controller.isTrajectoryRunning();
    }
    else if(cmd == RobotServer::COMMAND::PLACE_TRAY ||
            cmd == RobotServer::COMMAND::LOAD_TRAY ||
            cmd == RobotServer::COMMAND::INITIALIZE_TRAY)
    {
        return tray_controller.isActionRunning();
    }
    else
    {
        #ifdef PRINT_DEBUG
        Serial.print("Completion check not implimented for command: ");
        Serial.println(cmd);
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
        curCmd = RobotServer::COMMAND::NONE;
        statusUpdater.updateInProgress(false);
    }

    // Update loop time and status updater
    loop_time_averager.input(static_cast<float>(millis() - prevLoopMillis));
    prevLoopMillis = millis();
    statusUpdater.updateLoopTimes(static_cast<int>(loop_time_averager.mean()), static_cast<int>(position_time_averager.mean()));
    
}
