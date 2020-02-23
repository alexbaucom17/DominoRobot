#ifndef Globals_h
#define Globals_h

// Pins
#define PIN_PULSE_0 49
#define PIN_PULSE_1 51
#define PIN_PULSE_2 47
#define PIN_PULSE_3 45

#define PIN_DIR_0 48
#define PIN_DIR_1 50
#define PIN_DIR_2 46
#define PIN_DIR_3 44

#define PIN_ENABLE_ALL 40

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
#define STEPPER_PULSE_PER_REV 1600 // steps per revolution

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


#endif
