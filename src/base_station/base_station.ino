// Code for domino loading base station

#include "constants.h"
#include "BaseStationServer.h"
#include "StatusUpdater.h"
#include "BaseController.h"

StatusUpdater StatusUpdater;
BaseStationServer server = BaseStationServer(Serial3, Serial, StatusUpdater);
BaseController base_controller = BaseController(StatusUpdater, Serial);

void setup() 
{
    // Communication with the host computer
    Serial.begin(115200); 
    #ifdef PRINT_DEBUG
    Serial.println("Base station starting");
    #endif

    delay(100);
    server.begin();

    #ifdef PRINT_DEBUG
    Serial.println("Done with setup, starting loop");
    #endif
}

void runCmd(COMMAND cmd)
{

    if(cmd == COMMAND::LOAD)
    {
        base_controller.load();
    }
    else if (cmd == COMMAND::ESTOP)
    {
        base_controller.estop();
    }
    else if (cmd == COMMAND::NONE)
    {
      // do nothing...
    }
    else
    {
        #ifdef PRINT_DEBUG
        Serial.println("Unknown command!");
        #endif
    }

}

void loop()
{
    COMMAND newCmd = server.oneLoop();
    runCmd(newCmd);
    base_controller.update();
}