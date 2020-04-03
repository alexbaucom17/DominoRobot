#include "TrayController.h"
#include "constants.h"
#include <Arduino.h>

TrayController::TrayController(HardwareSerial& debug)
: debug_(debug),
  curAction_(ACTION::NONE),
  actionStep_(0),
  loadComplete_(false)
{
    latchServo_.attach(PIN_LATCH_SERVO_PIN);
    pinMode(PIN_TRAY_HOME_SWITCH, INPUT_PULLUP);
}

bool TrayController::isActionRunning()
{
    return curAction_ != ACTION::NONE;
}

void TrayController::initialize()
{
    curAction_ = ACTION::INITIALIZE;
    actionStep_ = 0;
    #ifdef PRINT_DEBUG
    debug_.println("Starting tray initialization");
    #endif 
}

void TrayController::place()
{
    curAction_ = ACTION::PLACE;
    actionStep_ = 0;
    #ifdef PRINT_DEBUG
    debug_.println("Starting tray placement");
    #endif 
}

void TrayController::load()
{
    curAction_ = ACTION::LOAD;
    actionStep_ = 0;
    #ifdef PRINT_DEBUG
    debug_.println("Starting tray loading");
    #endif 
}

void TrayController::estop()
{
    #ifdef PRINT_DEBUG
    debug_.println("Estopping tray control");
    #endif 
    curAction_ = ACTION::NONE;
}

void TrayController::update()
{
    switch (curAction_)
    {
    case ACTION::INITIALIZE:
        updateInitialize();
        break;
    case ACTION::LOAD:
        updateLoad();
        break;
    case ACTION::PLACE:
        updatePlace();
        break;
    default:
        break;
    }
}

void TrayController::updateInitialize()
{
    /*
    Sequence:
    0 - Close latch
    1 - Move tray until hit home
    2 - Reset pos, move tray to default location
    */

}

void TrayController::updatePlace()
{
    /*
    Sequence:
    0 - Move tray to place
    1 - Open latch
    2 - Wait
    3 - Close latch
    4 - Move tray to default
    */

}

void TrayController::updateLoad()
{
    /*
    Sequence:
    0 - Move tray to load
    1 - Wait for load complete signal
    2 - Move tray to default
    */

}