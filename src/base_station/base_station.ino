// Code for domino loading base station

#include "BaseStationServer.h"
#include "constants.h"

BaseStationServer server = BaseStationServer(Serial3, Serial);

// Variables used for loop
COMMAND newCmd = COMMAND::NONE;

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

void loop()
{

    // Check for new command and try to start it
    newCmd = server.oneLoop();

}