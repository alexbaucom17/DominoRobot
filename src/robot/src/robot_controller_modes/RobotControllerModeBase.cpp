#include "RobotControllerModeBase.h"

RobotControllerModeBase::RobotControllerModeBase(bool fake_perfect_motion)
: move_start_timer_(),
  move_running_(false),
  fake_perfect_motion_(fake_perfect_motion)
{
}

void RobotControllerModeBase::startMove()
{
    move_running_ = true;
    move_start_timer_.reset();
    loop_timer_.reset();
}