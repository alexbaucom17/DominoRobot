#include "TrayController.h"
#include "constants.h"
#include <plog/Log.h>
#include "serial/SerialCommsFactory.h"


TrayController::TrayController()
: serial_to_lifter_driver_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB)),
  action_step_running_(false),
  load_complete_(false),
  action_step_(0),
  cur_action_(ACTION::NONE),
  controller_rate_(cfg.lookup("tray.controller_frequency"))
{
}

void TrayController::initialize()
{
    cur_action_ = ACTION::INITIALIZE;
    action_step_ = 0;
    PLOGI << "Starting tray action INITIALIZE";
}

void TrayController::place()
{
    cur_action_ = ACTION::PLACE;
    action_step_ = 0;
    PLOGI << "Starting tray action PLACE";
}

void TrayController::load()
{
    cur_action_ = ACTION::LOAD;
    action_step_ = 0;
    PLOGI << "Starting tray action LOAD";
}

void TrayController::estop()
{
    cur_action_ = ACTION::NONE;
    PLOGW << "Estopping tray control";
    if (serial_to_lifter_driver_->isConnected())
    {
        serial_to_lifter_driver_->send("lift:stop");
    }
}

void TrayController::update()
{
    if (!controller_rate_.ready()) { return; }
    switch (cur_action_)
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
            // Just to make sure a serial buffer doesn't fill up somewhere
            std::string msg = "";
            msg = serial_to_lifter_driver_->rcv_lift();
            (void) msg;
            break;
    }
}

void TrayController::runStepAndWaitForCompletion(std::string data, std::string debug_print)
{
    if(!action_step_running_)
    {
        if (serial_to_lifter_driver_->isConnected())
        {
            serial_to_lifter_driver_->send(data);
            action_step_running_ = true;
            PLOGI << debug_print;
            action_timer_.reset();
        }
    }
    else
    {
        // Request status and wait for command to complete
        std::string msg = "";
        if (serial_to_lifter_driver_->isConnected())
        {
            serial_to_lifter_driver_->send("lift:status_req");
            msg = serial_to_lifter_driver_->rcv_lift();
        }
        // Initial delay for action
        if(action_timer_.dt_ms() > 1000 && msg == "none") {
            action_step_running_ = false;
            action_step_++;
        }
    }
}

void TrayController::updateInitialize()
{
    /*
    Sequence:
    0 - Close latch
    1 - Do tray init
    2 - Move to default location
    */

    // 0 - Close latch
    if(action_step_ == 0)
    {
        std::string data = "lift:close";
        std::string debug = "Closing latch";
        runStepAndWaitForCompletion(data, debug);
    }
    // 1 - Do tray init
    if(action_step_ == 1)
    {
        std::string data = "lift:home";
        std::string debug = "Homing tray";
        runStepAndWaitForCompletion(data, debug);
    }
    // 2 - Move to default location
    if(action_step_ == 2)
    {
        int pos = cfg.lookup("tray.default_pos_steps");
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to default position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 3 - Done with actinon
    if(action_step_ == 3)
    {
        cur_action_ = ACTION::NONE;
        PLOGI << "Done with initialization";
    }
}

void TrayController::updatePlace()
{
    /*
    Sequence:
    0 - Move tray to place
    1 - Open latch
    2 - Move tray to default
    3 - Close latch
    */

    // 0 - Move tray to place
    if(action_step_ == 0)
    {
        int pos = cfg.lookup("tray.place_pos_steps");
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to placement position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 1 - Open latch
    if(action_step_ == 1)
    {
        std::string data = "lift:open";
        std::string debug = "Opening latch";
        runStepAndWaitForCompletion(data, debug);
    }
    // 2 - Move tray to default
    if(action_step_ == 2)
    {
        int pos = cfg.lookup("tray.default_pos_steps");
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to default position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 3 - Close latch
    if(action_step_ == 3)
    {
        std::string data = "lift:close";
        std::string debug = "Closing latch";
        runStepAndWaitForCompletion(data, debug);
    }
    // 4 - Done with actinon
    if(action_step_ == 4)
    {
        cur_action_ = ACTION::NONE;
        PLOGI << "Done with place";
    }
}

void TrayController::updateLoad()
{
    /*
    Sequence:
    0 - Move tray to load
    1 - Wait for load complete signal
    2 - Move to default
    */

    // 0 - Move tray to load
    if(action_step_ == 0)
    {
        int pos = cfg.lookup("tray.load_pos_steps");
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to load position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 1 - Wait for load complete signal
    if(action_step_ == 1)
    {
        if(load_complete_)
        {
            action_step_++;
            PLOGI << "Got load complete signal";
            load_complete_ = false;
        }
    }
    // 2 - Move to default
    if(action_step_ == 2)
    {
        int pos = cfg.lookup("tray.default_pos_steps");
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to default position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 3 - Done with actinon
    if(action_step_ == 3)
    {
        cur_action_ = ACTION::NONE;
        PLOGI << "Done with load";
    }

}