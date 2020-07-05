#ifndef Globals_h
#define Globals_h

// Pinouts
#define PIN_ENABLE 52

#define PIN_ENCA_1 21
#define PIN_ENCA_2 20
#define PIN_ENCA_3 18
#define PIN_ENCA_4 19

#define PIN_ENCB_1 25
#define PIN_ENCB_2 24
#define PIN_ENCB_3 22
#define PIN_ENCB_4 23

#define PIN_DIR_1 39
#define PIN_DIR_2 13
#define PIN_DIR_3 12
#define PIN_DIR_4 37

#define PIN_PWM_1 5
#define PIN_PWM_2 6
#define PIN_PWM_3 7
#define PIN_PWM_4 4

// Velocities
#define MAX_WHEEL_SPEED 1.0          // rad/s - current motor can only do about 1 rev/s under load
#define MAX_TRANS_SPEED_FINE   0.08  // m/s
#define MAX_ROT_SPEED_FINE     0.5   // rad/2
#define MAX_TRANS_SPEED_COARSE 0.5  // m/s
#define MAX_ROT_SPEED_COARSE   1.0   // rad/2

// Accelerations
#define MAX_TRANS_ACC_FINE   0.1    // m/s^2
#define MAX_ROT_ACC_FINE     0.5    // rad/s^2
#define MAX_TRANS_ACC_COARSE 0.5    // m/s^2
#define MAX_ROT_ACC_COARSE   1.0    // rad/s^2

// Motor control gains
#define MOTOR_KP 70
#define MOTOR_KI 1
#define MOTOR_KD 0

// Cartesian control gains
#define CART_TRANS_KP 2
#define CART_TRANS_KI 0.1
#define CART_TRANS_KD 0
#define CART_ROT_KP 3
#define CART_ROT_KI 0.5
#define CART_ROT_KD 0

// Physical dimensions
#define WHEEL_RADIUS 0.1016 / 2 // meters
#define WHEEL_DIST_FROM_CENTER 0.3548 // meters

// Scaling factors
#define TRAJ_MAX_FRACTION 0.7  // Only generate a trajectory to this fraction of max speed to give motors headroom to compensate
#define FUDGE_FACTOR 0.55 // Fudgy scaling factor to use until I find where my actual scaling problem is. Scales how far the robot has actually moved when it thinks it has moved 1 meter

// Notes keys - saves space to use numbers instead of string
// If need to save more space could consider loading strings into program memory instead of RAM
// Or even just sending this key number and have the string display on the master end
#define NOTES_KEY_CONTROLLER_FREQ 1
#define NOTES_KEY_POSITION_FREQ 2

// Set debug printing, comment out to skip debug printing
//#define PRINT_DEBUG true


#endif
