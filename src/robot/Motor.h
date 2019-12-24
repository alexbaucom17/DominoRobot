#ifndef Motor_h
#define Motor_h

#include <Encoder.h>
#include <PID_v1.h>
#include <Filters.h>

/*const unsigned int COUNTS_PER_MOTOR_REV = 44; 
const unsigned int MOTOR_GEAR_RATIO = 40;
const double FUDGE_FACTOR = 1.5;
const double COUNTS_PER_SHAFT_REV = COUNTS_PER_MOTOR_REV * MOTOR_GEAR_RATIO * FUDGE_FACTOR;*/
#define VEL_FILTER_FREQ 7 // HZ
const double COUNTS_PER_SHAFT_REV = 2758; //Manually measured

class Motor
{
  public:

    /*
     * Motor object constructor
     * pwmPin - which pin the motor PWM is connected to
     * dirPin - which pin the motor direction is connected to
     * encPinA - which pin the encoder wire A is connected to. This should be a pin capable of handling interrupts
     * encPinB - which pin the encoder wire B is connected to. This should ideally (but not required) be a pin capable of handling interrupts
     * Kp - Proportional gain
     * Ki - Integral gain
     * Kd - Derrivative gain
     */
    Motor(int pwmPin, int dirPin, int encPinA, int encPinB, double Kp, double Ki, double Kd);
    
    /*
     * Set the desired velocity in revs/second
     */
    void setCommand(double vel);
    
    /*
     * Run the actual controller. Make sure this gets called reasonably quickly (i.e. every 20 ms or so)
     */
    void runLoop();

    /*
    * Get the current measured motor velocity
    */
    float getCurrentVelocity();

    // Get number of counts from encoder - for debugging
    long getCounts();

  private:

    // Pins
    int pwmPin_;
    int dirPin_;   

    // Values for computation
    double inputVel_;              // Desired velocity in revs/sec
    double currentVelRaw_;         // Raw current velocity in revs/sec
    double currentVelFiltered_;    // Filtered velocity in revs/sec
    double pidOut_;                // Output from PID controller
    int outputCmd_;             // Output command in [-255, 255]
    long prevCount_;               // Encoder count from previous loop

    // Timer
    unsigned long prevMillis_;   // Timer from previous loop

    // Other objects
    Encoder enc_;                  // Encoder object
    PID controller_;               // PID controller
    FilterOnePole velFilter_;      // Velocity lowpass filter
};

#endif Motor_h
