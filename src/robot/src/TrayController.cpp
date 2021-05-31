#include "TrayController.h"
#include "constants.h"
#include <plog/Log.h>
#include "serial/SerialCommsFactory.h"

enum class LifterPosType
{
    DEFAULT,
    PLACE,
    LOAD,
};

int getPos(LifterPosType pos_type)
{
    float revs = 0.0;
    switch(pos_type)
    {
        case LifterPosType::DEFAULT:
            revs = cfg.lookup("tray.default_pos_revs");
            break;
        case LifterPosType::PLACE:
            revs = cfg.lookup("tray.place_pos_revs");
            break;
        case LifterPosType::LOAD:
            revs = cfg.lookup("tray.load_pos_revs");
            break;
    }
    int steps_per_rev = cfg.lookup("tray.steps_per_rev");
    float steps = steps_per_rev * revs;
    return static_cast<int>(steps);
}

TrayController::TrayController()
: serial_to_lifter_driver_(SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB)),
  action_step_running_(false),
  load_complete_(false),
  action_step_(0),
  fake_tray_motion_(cfg.lookup("tray.fake_tray_motions")),
  cur_action_(ACTION::NONE),
  controller_rate_(cfg.lookup("tray.controller_frequency")),
  is_initialized_(false)
{
    if(fake_tray_motion_) PLOGW << "Fake tray motion enabled";
}

void TrayController::initialize()
{
    cur_action_ = ACTION::INITIALIZE;
    action_step_ = 0;
    PLOGI << "Starting tray action INITIALIZE";
}

bool TrayController::place()
{
    if(!is_initialized_ && !fake_tray_motion_) 
    {
        PLOGE << "Tray is not initialized, aborting action";
        return false;
    }
    
    cur_action_ = ACTION::PLACE;
    action_step_ = 0;
    PLOGI << "Starting tray action PLACE";
    return true;
}

bool TrayController::load()
{
    if(!is_initialized_ && !fake_tray_motion_) 
    {
        PLOGE << "Tray is not initialized, aborting action";
        return false;
    }
    
    cur_action_ = ACTION::LOAD;
    action_step_ = 0;
    PLOGI << "Starting tray action LOAD";
    return true;
}

void TrayController::estop()
{
    cur_action_ = ACTION::NONE;
    action_step_ = 0;
    action_step_running_ = false;
    PLOGW << "Estopping tray control";
    if (serial_to_lifter_driver_->isConnected())
    {
        serial_to_lifter_driver_->send("lift:stop");
    }
}

void TrayController::setLoadComplete()
{
    if(cur_action_ == ACTION::LOAD && action_step_ == 1)
    {
        load_complete_ = true;
    } 
    else 
    {
        PLOGW << "Recieved LOAD_COMPLETE signal at incorrect time. Ignoring.";
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
        }
        action_step_running_ = true;
        PLOGI << debug_print;
        action_timer_.reset();
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
        if(action_timer_.dt_ms() > 1000 && (msg == "none" || fake_tray_motion_)) {
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
        int pos = getPos(LifterPosType::DEFAULT);
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to default position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 3 - Done with actinon
    if(action_step_ == 3)
    {
        cur_action_ = ACTION::NONE;
        PLOGI << "Done with initialization";
        is_initialized_ = true;
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
        int pos = getPos(LifterPosType::PLACE);
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
        int pos = getPos(LifterPosType::DEFAULT);
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
        int pos = getPos(LifterPosType::LOAD);
        std::string data = "lift:pos:" + std::to_string(pos);
        std::string debug = "Moving tray to load position";
        runStepAndWaitForCompletion(data, debug);
    }
    // 1 - Wait for load complete signal
    if(action_step_ == 1)
    {
        if(!action_step_running_) 
        {
            action_step_running_ = true;
            PLOGI << "Waiting for load complete";
        } 
        else
        {
            if(load_complete_)
            {
                action_step_++;
                PLOGI << "Got load complete signal";
                load_complete_ = false;
                action_step_running_ = false;
            }
        }
    }
    // 2 - Move to default
    if(action_step_ == 2)
    {
        int pos = getPos(LifterPosType::DEFAULT);
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