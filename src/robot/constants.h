#ifndef Constants_h
#define Contsants_h

// Pins
// Note - according to https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/, pin 4 on the Mega has a different PWM frequency, need to make sure that doesn't affect things
#define PIN_SPEED_0 8   
#define PIN_SPEED_1 6
#define PIN_SPEED_2 5
#define PIN_SPEED_3 4
#define PIN_SPEED_DUMMY 7

#define PIN_DIR_0 22
#define PIN_DIR_1 24
#define PIN_DIR_2 26
#define PIN_DIR_3 28

#define PIN_ENABLE_ALL 29

#define PIN_LATCH_SERVO_PIN 12
#define PIN_TRAY_STEPPER_LEFT_PULSE 44
#define PIN_TRAY_STEPPER_LEFT_DIR 43
#define PIN_TRAY_STEPPER_RIGHT_PULSE 46
#define PIN_TRAY_STEPPER_RIGHT_DIR 47
#define PIN_TRAY_HOME_SWITCH 53

// Mapping from driver to motor - needed becuase it is easy to accidentally swap the drivers (with fixed pins) to a different motor
#define DRIVER_0_MOTOR 2
#define DRIVER_1_MOTOR 3
#define DRIVER_2_MOTOR 1
#define DRIVER_3_MOTOR 0

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

// Scaling factors
#define TRAJ_MAX_FRACTION 0.7  // Only generate a trajectory to this fraction of max speed to give motors headroom to compensate

// Scaling for stepper drive motors
#define STEPPER_PULSE_PER_REV 800 // steps per revolution
#define STEPPER_MAX_VEL 1600   // steps/second. This must match the value defined in stepper_driver.ino
#define PWM_RESOLUTION 255   // Analog resolution of arduino pwm output.

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

// Tray control values
#define TRAY_HOME_POS 1000     // Steps to move for homing, should be slightly larger than the max number of steps possible
#define TRAY_DEFAULT_POS 300   // Default position for driving in steps
#define TRAY_LOAD_POS 100      // Loading position in steps
#define TRAY_PLACE_POS 500     // Placing position in steps
#define TRAY_MAX_SPEED 200     // Max tray speed in steps/sec
#define TRAY_MAX_ACCEL 100     // Max tray acceleration in steps/sec/sec
#define LATCH_CLOSE_POS 20     // Latch servo position for close in degrees
#define LATCH_OPEN_POS 150     // Latch servo position for open in degrees
#define TRAY_PLACEMENT_PAUSE_TIME 3000 // How many ms to wait after opening the latch for placement
#define TRAY_SERVO_MIN_PW 1000  // Min pulse witdh in microseconds corresponding to 0 position
#define TRAY_SERVO_MAX_PW 2000  // Max pulse witdh in microseconds corresponding to 180 position


#endif
