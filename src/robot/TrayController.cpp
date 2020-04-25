#include "TrayController.h"
#include "constants.h"
#include <Arduino.h>

const float MM_TO_STEPS = TRAY_STEPPER_STEPS_PER_REV / TRAY_DIST_PER_REV;
const float TRAY_DEFAULT_POS = TRAY_DEFAULT_POS_MM * MM_TO_STEPS;
const float TRAY_LOAD_POS = TRAY_LOAD_POS_MM * MM_TO_STEPS;
const float TRAY_PLACE_POS = TRAY_PLACE_POS_MM * MM_TO_STEPS;
const float TRAY_MAX_STEPS = TRAY_MAX_LINEAR_TRAVEL * MM_TO_STEPS;

TrayController::TrayController(HardwareSerial& debug)
: debug_(debug),
  lifterLeft_(AccelStepper::DRIVER, PIN_TRAY_STEPPER_LEFT_PULSE, PIN_TRAY_STEPPER_LEFT_DIR),
  lifterRight_(AccelStepper::DRIVER, PIN_TRAY_STEPPER_RIGHT_PULSE, PIN_TRAY_STEPPER_RIGHT_DIR),
  curAction_(ACTION::NONE),
  actionStep_(0),
  loadComplete_(false),
  startMillisForTimer_(0)
{
}

void TrayController::begin()
{
    // Setup servo
    pinMode(PIN_LATCH_SERVO_PIN, OUTPUT);
    latchServo_.attach(PIN_LATCH_SERVO_PIN, TRAY_SERVO_MIN_PW, TRAY_SERVO_MAX_PW);
    latchServo_.write(LATCH_CLOSE_POS);

    // Setup home switch
    pinMode(PIN_TRAY_HOME_SWITCH, INPUT_PULLUP);

    // Setup lifters
    lifterLeft_.setMaxSpeed(TRAY_MAX_SPEED);
    lifterRight_.setMaxSpeed(TRAY_MAX_SPEED);
    lifterLeft_.setAcceleration(TRAY_MAX_ACCEL);
    lifterRight_.setAcceleration(TRAY_MAX_ACCEL);
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
    0 - Close latch and start tray move
    1 - Wait until we hit home, reset pos
    2 - start tray move to default location
    3 - Wait for tray move to complete
    */

    // 0 - Close latch and start tray move
    if(actionStep_ == 0)
    {
        // Close the tray
        latchServo_.write(LATCH_CLOSE_POS);

        // Start moving towards home pos with limit of max steps
        // Note the negative is so that all other moves are positive
        lifterLeft_.move(-1*TRAY_MAX_STEPS);
        lifterRight_.move(-1*TRAY_MAX_STEPS);
        actionStep_++;

        #ifdef PRINT_DEBUG
        debug_.println("TrayController - init started moving to home pos");
        #endif
    }

    // 1 - Wait until we hit home
    if(actionStep_ == 1)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();
        bool switchPressed = !digitalRead(PIN_TRAY_HOME_SWITCH); // Switch pin goes low when pressed

        // If we bumped the switch, then we are homed
        if(switchPressed)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            lifterLeft_.setCurrentPosition(0);
            lifterRight_.setCurrentPosition(0);
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - switch pressed");
            #endif
        }
        // If we didn't press the switch but one of the motors has stopped moving, something is wrong
        else if(!leftRunning || !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            debug_.println("ERROR!!! Switch press was not detected during tray homing");
            curAction_ = ACTION::NONE;
        }
    }

    // 2 - start tray move to default location
    if (actionStep_ == 2)
    {
        // Start moving towards default location
        lifterLeft_.moveTo(TRAY_DEFAULT_POS);
        lifterRight_.moveTo(TRAY_DEFAULT_POS);
        actionStep_++;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - moving to default pos");
        #endif
    }

    // 3 - Wait for tray move to complete
    if (actionStep_ == 3)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();

        // Wait for motors to stop
        if(!leftRunning && !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            curAction_ = ACTION::NONE;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - Done with tray initialization");
            #endif 
        }
    }
}

void TrayController::updatePlace()
{
    /*
    Sequence:
    0 - Start tray move to place
    1 - Wait for tray move to finish
    2 - Open latch
    3 - Wait for latch to fully open and dominoes to settle
    4 - Start tray move to default
    5 - Wait for tray move to finish
    6 - Close latch
    */

    // 0 - Start tray move to place
    if (actionStep_ == 0)
    {
        // Start moving towards place location
        lifterLeft_.moveTo(TRAY_PLACE_POS);
        lifterRight_.moveTo(TRAY_PLACE_POS);
        actionStep_++;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - moving to place pos");
        #endif
    }

    // 1 - Wait for tray move to finish
    if (actionStep_ == 1)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();

        // Wait for motors to stop
        if(!leftRunning && !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - movement complete");
            #endif
        }
    }

    // 2 - Open latch
    if (actionStep_ == 2)
    {
        latchServo_.write(LATCH_OPEN_POS);
        actionStep_++;
        startMillisForTimer_ = millis();
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - latch opening");
        #endif
    }

    // 3 - Wait for latch to fully open and dominoes to settle
    if (actionStep_ == 3)
    {
        if (millis() - startMillisForTimer_ > TRAY_PLACEMENT_PAUSE_TIME)
        {
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - latch opened");
            #endif
        }
    }

    // 4 - Start tray move to default
    if (actionStep_ == 4)
    {
        // Start moving towards default location
        lifterLeft_.moveTo(TRAY_DEFAULT_POS);
        lifterRight_.moveTo(TRAY_DEFAULT_POS);
        actionStep_++;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - moving to default pos");
        #endif
    }


    // 5 - Wait for tray move to finish
    if (actionStep_ == 5)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();

        // Wait for motors to stop
        if(!leftRunning && !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - movement complete");
            #endif
        }
    }

    // 6 - Close latch
    if(actionStep_ == 6)
    {
        latchServo_.write(LATCH_CLOSE_POS);
        curAction_ = ACTION::NONE;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - Done with tray placement");
        #endif 
    }

}

void TrayController::updateLoad()
{
    /*
    Sequence:
    0 - Start tray moving to load
    1 - Wait for tray move to finish
    2 - Wait for load complete signal
    3 - Start tray moving to default
    4 - Wait for tray move to finish
    */

    // 0 - Start tray move to load
    if (actionStep_ == 0)
    {
        // Start moving towards load location
        lifterLeft_.moveTo(TRAY_LOAD_POS);
        lifterRight_.moveTo(TRAY_LOAD_POS);
        actionStep_++;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - moving to load pos");
        #endif
    }

    // 1 - Wait for tray move to finish
    if (actionStep_ == 1)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();

        // Wait for motors to stop
        if(!leftRunning && !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - movement complete, awaiting load complete signal");
            #endif
        }
    }

    // 2 - Wait for load complete signal
    if (actionStep_ == 2)
    {
        if(loadComplete_)
        {
            loadComplete_ = false;
            actionStep_++;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - got load complete");
            #endif
        }
    }

    // 3 - Start tray move to default
    if (actionStep_ == 3)
    {
        // Start moving towards default location
        lifterLeft_.moveTo(TRAY_DEFAULT_POS);
        lifterRight_.moveTo(TRAY_DEFAULT_POS);
        actionStep_++;
        #ifdef PRINT_DEBUG
        debug_.println("TrayController - moving to default pos");
        #endif
    }


    // 4 - Wait for tray move to finish
    if (actionStep_ == 4)
    {
        // Run the motors
        bool leftRunning = lifterLeft_.run();
        bool rightRunning = lifterRight_.run();

        // Wait for motors to stop
        if(!leftRunning && !rightRunning)
        {
            lifterLeft_.stop();
            lifterRight_.stop();
            curAction_ = ACTION::NONE;
            #ifdef PRINT_DEBUG
            debug_.println("TrayController - Done with tray loading");
            #endif 
        }
    }

}
