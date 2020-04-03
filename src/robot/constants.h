#ifndef Globals_h
#define Globals_h

// Pins
#define PIN_PULSE_2 49
#define PIN_PULSE_3 51
#define PIN_PULSE_1 47
#define PIN_PULSE_0 45

#define PIN_DIR_2 48
#define PIN_DIR_3 50
#define PIN_DIR_1 46
#define PIN_DIR_0 44

#define PIN_ENABLE_ALL 40

#define PIN_LATCH_SERVO_PIN 10
#define PIN_TRAY_STEPPER_LEFT_PULSE 11
#define PIN_TRAY_STEPPER_LEFT_DIR 12
#define PIN_TRAY_STEPPER_RIGHT_PULSE 13
#define PIN_TRAY_STEPPER_RIGHT_DIR 14
#define PIN_TRAY_HOME_SWITCH 15

// Velocitiy limits
#define MAX_TRANS_SPEED_FINE   0.08  // m/s
#define MAX_ROT_SPEED_FINE     0.5   // rad/2
#define MAX_TRANS_SPEED_COARSE 0.5  // m/s
#define MAX_ROT_SPEED_COARSE   1.0   // rad/2

// Acceleration limits
#define MAX_TRANS_ACC_FINE   0.1    // m/s^2
#define MAX_ROT_ACC_FINE     0.5    // rad/s^2
#define MAX_TRANS_ACC_COARSE 0.5    // m/s^2
#define MAX_ROT_ACC_COARSE   1.0    // rad/s^2

// Cartesian control gains
#define CART_TRANS_KP 2
#define CART_TRANS_KI 0.1
#define CART_TRANS_KD 0
#define CART_ROT_KP 3
#define CART_ROT_KI 0.5
#define CART_ROT_KD 0

// Physical dimensions
#define WHEEL_DIAMETER 0.1016 // meters
#define WHEEL_DIST_FROM_CENTER 0.4572 // meters
#define STEPPER_PULSE_PER_REV 3200 // steps per revolution

// Scaling factors
#define TRAJ_MAX_FRACTION 0.7  // Only generate a trajectory to this fraction of max speed to give motors headroom to compensate

// Kalman filter scales
#define PROCESS_NOISE_SCALE 0.08
#define MEAS_NOISE_SCALE 0.01
#define MEAS_NOISE_VEL_SCALE_FACTOR 10000

// Possition accuracy targets
#define TRANS_POS_ERR_COARSE 0.10 // m
#define ANG_POS_ERR_COARSE   0.08 // rad
#define TRANS_POS_ERR_FINE   0.01 // m
#define ANG_POS_ERR_FINE     0.02 // rad

// Set debug printing, comment out to skip debug printing
#define PRINT_DEBUG true

// Tray control values
#define TRAY_HOME_POS 1000     // Steps to move for homing, should be slightly larger than the max number of steps possible
#define TRAY_DEFAULT_POS 300   // Default position for driving in steps
#define TRAY_LOAD_POS 100      // Loading position in steps
#define TRAY_PLACE_POS 500     // Placing position in steps
#define TRAY_MAX_SPEED 100     // Max tray speed in steps/sec
#define TRAY_MAX_ACCEL 10      // Max tray acceleration in steps/sec/sec
#define LATCH_CLOSE_POS 10     // Latch servo position for close in degrees
#define LATCH_OPEN_POS 100     // Latch servo position for open in degrees
#define TRAY_PLACEMENT_PAUSE_TIME 3000 // How many ms to wait after opening the latch for placement


#endif
