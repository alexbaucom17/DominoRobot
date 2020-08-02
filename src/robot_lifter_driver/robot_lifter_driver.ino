// Uses stepper driver library from here: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <AccelStepper.h>
#include <SerialComms.h>

// LEFT/RIGHT is WRT standing at back of robot looking forward
#define STEP_PIN_LEFT 2
#define STEP_PIN_RIGHT 3
#define DIR_PIN_LEFT 4
#define DIR_PIN_RIGHT 5
#define INCREMENTAL_UP_PIN 6
#define INCREMENTAL_DOWN_PIN 7
#define HOMING_SWITCH_PIN 11

#define MAX_VEL 10  // steps/sec
#define MAX_ACC 100  // steps/sec^2

#define HOMING_DIST 10000 // This should be larger than the max distance the lifter can go
#define SAFETY_MAX_POS 1000  // Sanity check on desired position to make sure it isn't larger than this
#define SAFETY_MIN_POS -1000 // Sanity check on desired position to make sure it isn't less than this

AccelStepper motors[2];
SerialComms comms(Serial);
bool move_in_progress = false;
bool mode_vel_active = false;
bool mode_pos_active = false;
bool mode_homing_active = false;

struct Command
{
    int abs_pos;
    bool home;
    bool valid;
    bool stop;
};

void setup() 
{
    pinMode(INCREMENTAL_UP_PIN, INPUT_PULLUP);
    pinMode(INCREMENTAL_DOWN_PIN, INPUT_PULLUP);
    pinMode(HOMING_SWITCH_PIN, INPUT_PULLUP);
    
    prevMillisRead = millis();
    
    motors[0] = AccelStepper(AccelStepper::DRIVER, STEP_PIN_LEFT, DIR_PIN_LEFT);
    motors[1] = AccelStepper(AccelStepper::DRIVER, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

    for(int i = 0; i < 2; i++)
    {
      motors[i].setMaxSpeed(MAX_VEL);
      motors[i].setAcceleration(MAX_ACC);
      motors[i].setMinPulseWidth(10); // Min from docs is 2.5 microseconds
    }
}


// Possible inputs are
// "home", "stop", or an integer representing position to move to
Command getAndDecodeMsg()
{
    String msg = comms.rcv();
    Command c = {0, false, false, false};
    if(!msg.empty())
    {
        if(msg == "home")
        {
            c.home = true;
            c.valid = true;
        }
        else if(msg == "stop")
        {
            c.stop = true;
            c.valid = true;
        }
        else
        {
            c.abs_pos = msg.toInt();
            c.valid = true;
        }
    }

    return c;
}

void loop()
{

    // Check for an incomming command
    Command inputCommand = getAndDecodeMsg();
    
    // Read inputs for vel
    bool vel_up = !digitalRead(INCREMENTAL_UP_PIN);
    bool vel_down = !digitalRead(INCREMENTAL_DOWN_PIN);

    // If we got a stop command, make sure to handle it immediately
    if (inputCommand.valid && inputCommand.stop)
    {
        mode_pos_active = false;
        mode_vel_active = false;
        mode_homing_active = false;
    }
    // For any other command, we will only handle it when there are no other active commands
    else if (!(mode_pos_active || mode_vel_active || mode_homing_active))
    {
        // Figure out what mode we are in
        if(vel_up || vel_down)
        {
            mode_vel_active = true;
        }
        else if(inputCommand.valid && inputCommand.home)
        {
            mode_homing_active = true;
            
        }
        else if(inputCommand.valid)
        {
            if(inputCommand.abs_pos < SAFETY_MAX_POS && inputCommand.abs_pos > SAFETY_MIN_POS)
            {
                mode_pos_active = true;
            }
        }
    }

    
    // Set motor speeds based on inputs and mode
    if(mode_vel_active)
    {
        if(vel_up)
        {
            motors[0].setSpeed(MAX_VEL);
            motors[1].setSpeed(MAX_VEL);
        }
        else if(vel_down)
        {
            motors[0].setSpeed(-1*MAX_VEL);
            motors[1].setSpeed(-1*MAX_VEL);
        }     
    }
    else if(mode_pos_active && !move_in_progress)
    {
        int target = inputCommand.abs_pos;
        motors[0].moveTo(target);
        motors[1].moveTo(target);
        move_in_progress = true;
    }
    else if(mode_homing_active && !move_in_progress)
    {
        motors[0].moveTo(HOMING_DIST);
        motors[1].moveTo(HOMING_DIST);
        move_in_progress = true;
    }
    else if(!move_in_progress)
    {
        // Stop motors if no move is in progress and no buttons are pushed
        motors[0].stop();
        motors[1].stop();        
    }


    // Call motor update loop
    String status_str = "none";
    if(mode_vel_active)
    {
        motors[0].runSpeed();
        motors[1].runSpeed();
        status_str = "manaul";
    }
    else if(mode_pos_active)
    {
        bool tmp1 = motors[0].run();
        bool tmp2 = motors[1].run();
        move_in_progress = tmp1 || tmp2;
        status_str = "pos";
    }
    else if(mode_homing_active)
    {
        motors[0].run();
        motors[1].run();
        if(!digitalRead(HOMING_SWITCH_PIN))
        {
            motors[0].stop();
            motors[1].stop();  
            move_in_progress = false;
            motors[0].setCurrentPosition(0);
            motors[1].setCurrentPosition(0);
        }
        status_str = "homing";
    }

    comms.send(status_str);
}
