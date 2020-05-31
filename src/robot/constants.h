#ifndef Constants_h
#define Constants_h

// Pins
#define PIN_PWM_0 4   // FL
#define PIN_PWM_1 3   // FR
#define PIN_PWM_2 5   // BR
#define PIN_PWM_3 6   // BL

#define PIN_DIR_0 40
#define PIN_DIR_1 41
#define PIN_DIR_2 39
#define PIN_DIR_3 38

#define PIN_ENABLE_0 48
#define PIN_ENABLE_1 49
#define PIN_ENABLE_2 47
#define PIN_ENABLE_3 46

#define PIN_ENC_A_0 20
#define PIN_ENC_B_0 23
#define PIN_ENC_A_1 21
#define PIN_ENC_B_1 25
#define PIN_ENC_A_2 19
#define PIN_ENC_B_2 24
#define PIN_ENC_A_3 18
#define PIN_ENC_B_3 22

#define PIN_LATCH_SERVO_PIN 12
#define PIN_TRAY_STEPPER_LEFT_PULSE 46
#define PIN_TRAY_STEPPER_LEFT_DIR 47
#define PIN_TRAY_STEPPER_RIGHT_PULSE 44
#define PIN_TRAY_STEPPER_RIGHT_DIR 43
#define PIN_TRAY_HOME_SWITCH 53

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

// Motor control gains
#define MOTOR_KP 14
#define MOTOR_KI 0.1
#define MOTOR_KD 0.08

// Motor control constants
#define VEL_FILTER_FREQ 20            // HZ
#define COUNTS_PER_OUTPUT_SHAFT_REV 36000 // Manually measrued/estimated

// Physical dimensions
#define WHEEL_DIAMETER 0.152 // meters
#define WHEEL_DIST_FROM_CENTER 0.4794 // meters

// Scaling factors
#define TRAJ_MAX_FRACTION 0.7  // Only generate a trajectory to this fraction of max speed to give motors headroom to compensate
#define FUDGE_FACTOR 2.0 // Scaling factor for compensating for some math error I can't find

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

// Set debug printing, comment out to skip debug printing
#define PRINT_DEBUG true

// Tray stepper control values
#define TRAY_STEPPER_STEPS_PER_REV 200  // Steps per rev for tray servos
#define TRAY_DIST_PER_REV 1.8           // mm of linear travel per stepper revolution
#define TRAY_MAX_LINEAR_TRAVEL 300      // mm of total linear travel possible
// Note: all tray positions measured in mm from home pos
#define TRAY_DEFAULT_POS_MM 100         // Default position for driving in mm
#define TRAY_LOAD_POS_MM 50             // Loading position in mm
#define TRAY_PLACE_POS_MM 250           // Placing position in mm
#define TRAY_MAX_SPEED 1000             // Max tray speed in steps/sec
#define TRAY_MAX_ACCEL 2000             // Max tray acceleration in steps/sec/sec

// Tray servo control values
#define LATCH_CLOSE_POS 20              // Latch servo position for close in degrees
#define LATCH_OPEN_POS 150              // Latch servo position for open in degrees
#define TRAY_SERVO_MIN_PW 1000          // Min pulse witdh in microseconds corresponding to 0 position
#define TRAY_SERVO_MAX_PW 2000          // Max pulse witdh in microseconds corresponding to 180 position

#define TRAY_PLACEMENT_PAUSE_TIME 3000  // How many ms to wait after opening the latch for placement

enum COMMAND
{
    NONE,
    MOVE,
    MOVE_REL,
    MOVE_FINE,
    PLACE_TRAY,
    LOAD_TRAY,
    INITIALIZE_TRAY,
    POSITION,
    ESTOP,
    LOAD_COMPLETE,
};

#endif
