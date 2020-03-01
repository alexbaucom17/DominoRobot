/* Requires libraries:
*  Filters: https://github.com/JonHub/Filters
*  PID: https://playground.arduino.cc/Code/PIDLibrary/
*  ArduinoJson: https://arduinojson.org
*  LinAlgebra: https://github.com/dams666/LinAlgebra
*   (modified to reduce matrix size to 3)
*  ArduinoSTL: https://www.arduinolibraries.info/libraries/arduino-stl
*  MemoryFree: https://playground.arduino.cc/Code/AvailableMemory/
*  StepperDriver: https://github.com/DIMRobotics/ArduinoStepperDriver
*   (modified to increase stepper count to 4)
*/

#include "RobotServer.h"
#include "RobotController.h"
#include "StatusUpdater.h"

StatusUpdater statusUpdater;
RobotServer server = RobotServer(Serial3, Serial, statusUpdater);
RobotController controller = RobotController(Serial, statusUpdater);


void setup()
{
    // Communication with the host computer
    Serial.begin(115200); 
    #ifdef PRINT_DEBUG
    Serial.println("Robot starting");
    #endif

    // Need this delay for controller to setup correctly for some reason
    delay(100);

    // Start server and controller
    
    controller.begin();
    delay(100);
    server.begin();
    

    Serial.println("Done with setup");
}


void loop() 
{
    // Check for new command
    RobotServer::COMMAND newCmd = server.oneLoop();

    // Handle new command
    if(newCmd == RobotServer::COMMAND::MOVE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPosition(data.x, data.y, data.a);
    }
    else if(newCmd == RobotServer::COMMAND::MOVE_REL)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionRelative(data.x, data.y, data.a);
    }
    else if(newCmd == RobotServer::COMMAND::MOVE_FINE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionFine(data.x, data.y, data.a);
    }
    else if (newCmd == RobotServer::COMMAND::POSITION)
    {
        RobotServer::PositionData data = server.getPositionData();
        controller.inputPosition(data.x, data.y, data.a);
    }

    // Service controller
    controller.update();
    
}
