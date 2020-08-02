#include "TrayController.h"
#include "constants.h"
#include <plog/Log.h>


TrayController::TrayController()
: serial_to_lifter_driver_(buildSerialComms(LIFTER_DRIVER_USB)),
  action_running_(false),
  load_complete_(false),
  cur_action_(ACITON::NONE)
{
}

void TrayController::initialize()
{
    if (serial_to_lifter_driver_->isConnected())
    {
        serial_to_lifter_driver_->send("home");
    }
    cur_action = ACTION::INITIALIZE;
}

void TrayController::place()
{
    if (serial_to_lifter_driver_->isConnected())
    {
        
        serial_to_lifter_driver_->send("home");
    }
    cur_action = ACTION::INITIALIZE;
}

void TrayController::load()

void TrayController::update()
{
    std::string msg = "";
    if (serial_to_lifter_driver_->isConnected())
    {
         msg = serial_to_lifter_driver_->rcv();
    }
    
    if (msg.empty())
    {
        return;
    }
    else
    {
        if (msg == "none")
        {
            action_running_ = false;
        }
        else if (msg != EXPECTED_MSG[cur_action_])
        {
            PLOGW << "Got message: " << msg << ", but expected: " << EXPECTED_MSG[cur_action_];
        }
    }
}

void TrayController::estop()
