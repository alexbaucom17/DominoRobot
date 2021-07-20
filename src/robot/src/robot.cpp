#include "robot.h"

#include <plog/Log.h> 
#include "utils.h"
#include "camera_tracker/CameraTrackerFactory.h"


WaitForLocalizeHelper::WaitForLocalizeHelper(const StatusUpdater& statusUpdater, float max_timeout, float confidence_threshold) 
: statusUpdater_(statusUpdater),
    timer_(),
    max_timeout_(max_timeout),
    confidence_threshold_(confidence_threshold)
{}

bool WaitForLocalizeHelper::isDone() 
{
    if(timer_.dt_s() > max_timeout_)
    {
        PLOGW.printf("Exiting wait for localize due to time");
        return true;
    }
    float confidence = statusUpdater_.getLocalizationConfidence();
    if(confidence > confidence_threshold_)
    {
        PLOGI.printf("Exiting wait for localize with confidence: %4.2f", confidence);
        return true;
    }
    return false;
}

void WaitForLocalizeHelper::start()
{
    timer_.reset();
}



Robot::Robot()
: statusUpdater_(),
  server_(statusUpdater_),
  controller_(statusUpdater_),
  tray_controller_(),
  mm_wrapper_(),
  position_time_averager_(10),
  robot_loop_time_averager_(20),
  wait_for_localize_helper_(statusUpdater_, cfg.lookup("localization.max_wait_time"), cfg.lookup("localization.confidence_for_wait")),
  vision_print_rate_(10),
  camera_tracker_(CameraTrackerFactory::getFactoryInstance()->get_camera_tracker()),
  camera_motion_start_time_(ClockTimePoint::min()),
  camera_trigger_time_1_(ClockTimePoint::min()),
  camera_trigger_time_2_(ClockTimePoint::min()),
  camera_stop_triggered_(false),
  curCmd_(COMMAND::NONE)
{
    PLOGI.printf("Robot starting");
}

void Robot::run()
{
    while(true)
    {
        runOnce();
    }
}


void Robot::runOnce() 
{
    // Check for new command and try to start it
    COMMAND newCmd = server_.oneLoop();
    bool status = tryStartNewCmd(newCmd);

    // Update our current command if we successfully started a new command
    if(status)
    {
        curCmd_ = newCmd;
        statusUpdater_.updateInProgress(true);
    }

    // Service marvelmind
    std::vector<float> positions = mm_wrapper_.getPositions();
    if (positions.size() == 3)
    {
        position_time_averager_.mark_point();
        float angle_rad = wrap_angle(positions[2] * M_PI / 180.0);
        controller_.inputPosition(positions[0], positions[1], angle_rad);
    }

    // Service various modules
    controller_.update();
    tray_controller_.update();
    camera_tracker_->update();

    // Verify if MOVE_FINE_STOP_VISION needs to trigger stop
    if(checkForCameraStopTrigger())
    {
        controller_.stopFast();
        camera_stop_triggered_ = true;
    }

    // Check if the current command has finished
    bool done = checkForCmdComplete(curCmd_);
    if(done)
    {
        curCmd_ = COMMAND::NONE;
        statusUpdater_.updateInProgress(false);
    }

    // Update loop time and status updater
    statusUpdater_.updatePositionLoopTime(position_time_averager_.get_ms());
    CameraDebug camera_debug = camera_tracker_->getCameraDebug();
    statusUpdater_.updateCameraDebug(camera_debug);
    robot_loop_time_averager_.mark_point();
}


bool Robot::tryStartNewCmd(COMMAND cmd)
{
    // Position info doesn't count as a real 'command' since it doesn't interrupt anything
    // Always service it, but don't consider it starting a new command
    if (cmd == COMMAND::POSITION)
    {
        RobotServer::PositionData data = server_.getPositionData();
        controller_.inputPosition(data.x, data.y, data.a);

        // Update the position rate
        position_time_averager_.mark_point();

        return false;
    }
    if (cmd == COMMAND::SET_POSE)
    {
        RobotServer::PositionData data = server_.getPositionData();
        controller_.forceSetPosition(data.x, data.y, data.a);
        return false;
    }
    if (cmd == COMMAND::TOGGLE_VISION_DEBUG)
    {
        camera_tracker_->toggleDebugImageOutput();
        return false;
    }
    if (cmd == COMMAND::START_CAMERAS)
    {
        camera_tracker_->start();
        return false;
    }
    if (cmd == COMMAND::STOP_CAMERAS)
    {
        camera_tracker_->stop();
        return false;
    }
    // Same with ESTOP
    if (cmd == COMMAND::ESTOP)
    {
        controller_.estop();
        tray_controller_.estop();
        return false;
    }
    // Same with LOAD_COMPLETE
    if (cmd == COMMAND::LOAD_COMPLETE)
    {
        tray_controller_.setLoadComplete();
        return false;
    }

    // Just do nothing for NONE
    if (cmd == COMMAND::NONE) { return false;}
    
    // For all other commands, we need to make sure we aren't doing anything else at the moment
    if(statusUpdater_.getInProgress())
    {
        PLOGW << "Command " << static_cast<int>(curCmd_) << " already running, rejecting new command: " << static_cast<int>(cmd);
        return false;
    }
    else if (statusUpdater_.getErrorStatus())
    {
        return false;
    }
    
    // Start new command
    if(cmd == COMMAND::MOVE)
    {
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveToPosition(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_REL)
    {
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveToPositionRelative(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_REL_SLOW)
    {
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveToPositionRelativeSlow(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_FINE)
    {
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveToPositionFine(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::MOVE_FINE_STOP_VISION)
    {
        if(!camera_tracker_->running()) 
        {
            PLOGW << "Cannot start MOVE_FINE_STOP_VISION if camera tracker isn't running";
            return false;
        }
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveToPositionFine(data.x, data.y, data.a);
        resetCameraStopTriggers();
        camera_motion_start_time_ = ClockFactory::getFactoryInstance()->get_clock()->now();
        fine_move_target_ = {data.x, data.y, data.a};
    
    }
    else if(cmd == COMMAND::MOVE_CONST_VEL)
    {
        RobotServer::VelocityData data = server_.getVelocityData();
        controller_.moveConstVel(data.vx, data.vy, data.va, data.t);
    }
    else if (cmd == COMMAND::MOVE_WITH_VISION)
    {
        RobotServer::PositionData data = server_.getMoveData();
        controller_.moveWithVision(data.x, data.y, data.a);
    }
    else if(cmd == COMMAND::PLACE_TRAY)
    {
        bool ok = tray_controller_.place();
        if(!ok) statusUpdater_.setErrorStatus();
        return ok;
    }
    else if(cmd == COMMAND::LOAD_TRAY)
    {
        bool ok = tray_controller_.load();
        if(!ok) statusUpdater_.setErrorStatus();
        return ok;
    }
    else if(cmd == COMMAND::INITIALIZE_TRAY)
    {
        tray_controller_.initialize();
    }
    else if (cmd == COMMAND::WAIT_FOR_LOCALIZATION)
    {
        wait_for_localize_helper_.start();
    }
    else
    {
        PLOGW.printf("Unknown command!");
        return false;
    }

    return true;
}

bool Robot::checkForCmdComplete(COMMAND cmd)
{
    if (cmd == COMMAND::NONE)
    {
        return true;
    }
    else if(cmd == COMMAND::MOVE || 
            cmd == COMMAND::MOVE_REL ||
            cmd == COMMAND::MOVE_FINE ||
            cmd == COMMAND::MOVE_CONST_VEL ||
            cmd == COMMAND::MOVE_WITH_VISION || 
            cmd == COMMAND::MOVE_REL_SLOW || 
            cmd == COMMAND::MOVE_FINE_STOP_VISION)
    {
        return !controller_.isTrajectoryRunning();
    }
    else if(cmd == COMMAND::PLACE_TRAY ||
            cmd == COMMAND::LOAD_TRAY ||
            cmd == COMMAND::INITIALIZE_TRAY)
    {
        return !tray_controller_.isActionRunning();
    }
    else if (cmd == COMMAND::WAIT_FOR_LOCALIZATION) 
    {
        return wait_for_localize_helper_.isDone();
    }
    else
    {
        PLOGE.printf("Completion check not implimented for command: %i",cmd);
        return true;
    }
        
}

bool Robot::checkForCameraStopTrigger()
{
    if(curCmd_ != COMMAND::MOVE_FINE_STOP_VISION) return false;
    if(camera_stop_triggered_) return false;

    CameraTrackerOutput camera_output = camera_tracker_->getPoseFromCamera();
    if(camera_output.ok)
    {
        bool camera_trigger_1_ = camera_trigger_time_1_ > camera_motion_start_time_;
        bool camera_trigger_2 = camera_trigger_time_2_ > camera_motion_start_time_;
        bool timestamp_after_1 = camera_output.timestamp > camera_trigger_time_1_;
        bool timestamp_after_2 = camera_output.timestamp > camera_trigger_time_2_;
        bool timestamp_difference = std::chrono::duration_cast<FpSeconds>(camera_output.timestamp - camera_trigger_time_1_).count() > 0;
        Point current_pos = controller_.getCurrentPosition();
        Eigen::Vector2f dp = {fine_move_target_.x - current_pos.x, fine_move_target_.y - current_pos.y};
        bool pos_in_tolerance = dp.norm() < 0.5;

        if(camera_trigger_1_ && camera_trigger_2 && timestamp_after_1 && timestamp_after_2 && pos_in_tolerance)
        {
            return true;
        }
        else if(camera_trigger_1_ && timestamp_after_1 && timestamp_difference)
        {
            camera_trigger_time_2_ = camera_output.timestamp;
            return true;
        }
        else
        {
            camera_trigger_time_1_ = camera_output.timestamp;
            return false;
        }
    }
    else
    {
        resetCameraStopTriggers();
    }
    return false;
}

void Robot::resetCameraStopTriggers()
{
    camera_stop_triggered_ = false;
    camera_trigger_time_1_ = ClockTimePoint::min();
    camera_trigger_time_2_ = ClockTimePoint::min();
}
