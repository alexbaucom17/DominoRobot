#ifndef Constants_h
#define Constants_h

#include <libconfig.h++>
extern libconfig::Config cfg;

#define CONSTANTS_FILE "/home/pi/DominoRobot/src/robot/src/constants.cfg"
#define TEST_CONSTANTS_FILE "/home/pi/DominoRobot/src/robot/test/test_constants.cfg"

// Kalman filter scales
#define PROCESS_NOISE_SCALE 0.08
#define MEAS_NOISE_SCALE 0.01
#define MEAS_NOISE_VEL_SCALE_FACTOR 10000

// USB devices
#define CLEARCORE_USB "/dev/clearcore"
#define MARVELMIND_USB_0 "/dev/marvelmind0" //Marvelminds could show up at any of these three links
#define MARVELMIND_USB_1 "/dev/marvelmind1"
#define MARVELMIND_USB_2 "/dev/marvelmind2"

// Expected marvelmind device connections
#define MARVELMIND_DEVICE_ID0 5
#define MARVELMIND_DEVICE_ID1 6

// Log file ID for motion specific stuff
#define MOTION_LOG_ID 2
#define LOCALIZATION_LOG_ID 3

// Commands use to communicate about behavior specified from master
enum class COMMAND
{
    NONE,
    MOVE,
    MOVE_REL,
    MOVE_FINE,
    MOVE_CONST_VEL,
    PLACE_TRAY,
    LOAD_TRAY,
    INITIALIZE_TRAY,
    POSITION,
    ESTOP,
    LOAD_COMPLETE,
    CLEAR_ERROR,
    WAIT_FOR_LOCALIZATION,
    SET_POSE,
};

#endif
