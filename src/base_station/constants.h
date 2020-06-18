#ifndef Constants_h
#define Constants_h

// Pins
#define PIN_SENSOR_1 1
#define PIN_SENSOR_2 2
#define PIN_SENSOR_3 3
#define PIN_SENSOR_4 4

#define PIN_MOTOR_PWM 5

// Motor values
#define PWM_STILL 150
#define PWM_FWD 100
#define PWM_BKWD 200

#define LOAD_FWD_TIME_MS 1000
#define LOAD_PAUSE_TIME_MS 1000
#define LOAD_BKWD_TIME_MS 1000

// Set debug printing, comment out to skip debug printing
#define PRINT_DEBUG true

enum COMMAND
{
    NONE,
    ESTOP,
    LOAD,
};

#endif