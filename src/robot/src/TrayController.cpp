#include "TrayController.h"
#include "constants.h"

const float MM_TO_STEPS = TRAY_STEPPER_STEPS_PER_REV / TRAY_DIST_PER_REV;
const float TRAY_DEFAULT_POS = TRAY_DEFAULT_POS_MM * MM_TO_STEPS;
const float TRAY_LOAD_POS = TRAY_LOAD_POS_MM * MM_TO_STEPS;
const float TRAY_PLACE_POS = TRAY_PLACE_POS_MM * MM_TO_STEPS;
const float TRAY_MAX_STEPS = TRAY_MAX_LINEAR_TRAVEL * MM_TO_STEPS;

TrayController::TrayController()
: 
  curAction_(ACTION::NONE),
  actionStep_(0),
  loadComplete_(false),
  startMillisForTimer_(0)
{
    logger_ = spdlog::get("robot_logger")
}

// TODO: Convert this file for using arduino mini (copy action sequencing from previous versions of this file)
void TrayController::begin()
{
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
    logger_->info("Starting tray initialization");
    #endif 
}

void TrayController::place()
{
    curAction_ = ACTION::PLACE;
    actionStep_ = 0;
    #ifdef PRINT_DEBUG
    logger_->info("Starting tray placement");
    #endif 
}

void TrayController::load()
{
    curAction_ = ACTION::LOAD;
    actionStep_ = 0;
    #ifdef PRINT_DEBUG
    logger_->info("Starting tray loading");
    #endif 
}

void TrayController::estop()
{
    #ifdef PRINT_DEBUG
    logger_->info("Estopping tray control");
    #endif 
    curAction_ = ACTION::NONE;
}

void TrayController::update()
{
    switch (curAction_)
    {
    case ACTION::INITIALIZE:
        // updateInitialize();
        break;
    case ACTION::LOAD:
        // updateLoad();
        break;
    case ACTION::PLACE:
        // updatePlace();
        break;
    default:
        break;
    }

}