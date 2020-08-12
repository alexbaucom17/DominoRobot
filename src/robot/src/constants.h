#ifndef Constants_h
#define Constants_h

#include <libconfig.h++>
static libconfig::Config cfg;

// Velocitiy limits
#define MAX_TRANS_SPEED_FINE   0.08  // m/s
#define MAX_ROT_SPEED_FINE     0.5   // rad/2
#define MAX_TRANS_SPEED_COARSE 0.2   // m/s
#define MAX_ROT_SPEED_COARSE   1.0   // rad/2

// Acceleration limits
#define MAX_TRANS_ACC_FINE   0.1    // m/s^2
#define MAX_ROT_ACC_FINE     0.5    // rad/s^2
#define MAX_TRANS_ACC_COARSE 0.5    // m/s^2
#define MAX_ROT_ACC_COARSE   1.0    // rad/s^2

// Jerk limits
#define MAX_TRANS_JERK_FINE   0.5    // m/s^3
#define MAX_ROT_JERK_FINE     1.0    // rad/s^3
#define MAX_TRANS_JERK_COARSE 1.0    // m/s^3
#define MAX_ROT_JERK_COARSE   2.0    // rad/s^3

// Cartesian control gains
#define CART_TRANS_KP 2
#define CART_TRANS_KI 0.1
#define CART_TRANS_KD 0
#define CART_ROT_KP 3
#define CART_ROT_KI 0.5
#define CART_ROT_KD 0

// Physical dimensions
#define WHEEL_DIAMETER 0.152 // meters
#define WHEEL_DIST_FROM_CENTER 0.4794 // meters

// Trajectory generation
#define TRAJ_MAX_FRACTION 0.8   // Only generate a trajectory to this fraction of max speed to give motors headroom to compensate
#define SOLVER_MAX_LOOPS 10     // Only let the solver loop this many times before giving up
#define SOLVER_ALPHA_DECAY 0.8  // Decay for velocity limit
#define SOLVER_BETA_DECAY 0.8   // Decay for acceleration limit

// Kalman filter scales
#define PROCESS_NOISE_SCALE 0.08
#define MEAS_NOISE_SCALE 0.01
#define MEAS_NOISE_VEL_SCALE_FACTOR 10000

// Possition accuracy targets
#define TRANS_POS_ERR_COARSE 0.10 // m
#define ANG_POS_ERR_COARSE   0.08 // rad
#define TRANS_VEL_ERR_COARSE 0.05 // m/s
#define ANG_VEL_ERR_COARSE   0.05 // rad/s
#define TRANS_POS_ERR_FINE   0.01 // m
#define ANG_POS_ERR_FINE     0.02 // rad
#define TRANS_VEL_ERR_FINE   0.01 // m/s
#define ANG_VEL_ERR_FINE     0.01 // rad/s

// Tray positions
#define TRAY_DEFAULT_POS_STEPS 10         // Default position for driving in steps from home
#define TRAY_LOAD_POS_STEPS 5             // Loading position in steps from home
#define TRAY_PLACE_POS_STEPS 25           // Placing position in steps from home

// USB devices
#define CLEARCORE_USB "/dev/clearcore"
#define LIFTER_DRIVER_USB "/dev/arduino"
#define MARVELMIND_USB_0 "/dev/marvelmind0" //Marvelminds could show up at any of these three links
#define MARVELMIND_USB_1 "/dev/marvelmind1"
#define MARVELMIND_USB_2 "/dev/marvelmind2"

// Expected marvelmind device connections
#define MARVELMIND_DEVICE_ID0 5
#define MARVELMIND_DEVICE_ID1 6

#define MOTION_LOG_ID 2

enum COMMAND
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
};

// extern const ConstantParams CONSTANTS;


// struct ConstantParams
// {
//     float MAX_TRANS_SPEED_FINE;
//     float MAX_ROT_SPEED_FINE;
//     float MAX_TRANS_SPEED_COARSE;
//     float MAX_ROT_SPEED_COARSE;
// };

#endif
