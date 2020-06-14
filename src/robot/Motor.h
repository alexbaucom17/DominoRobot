#ifndef Motor_h
#define Motor_h

#include <Encoder.h>
#include <PID_v1.h>
#include <Filters.h>

class Motor
{
  public:

    /*
     * Motor object constructor
     * pwmPin - which pin the motor PWM is connected to
     * dirPin - which pin the motor direction is connected to
     * encPinA - which pin the encoder wire A is connected to. This should be a pin capable of handling interrupts
     * encPinB - which pin the encoder wire B is connected to. This should ideally (but not required) be a pin capable of handling interrupts
     */
    Motor(int pwmPin, int dirPin, int encPinA, int encPinB);
    
    /*
     * Set the desired velocity in rad/second
     */
    void setCommand(double vel);

    /* 
    * Set the motor control gains
    */ 
    void setGains(double Kp, double Ki, double Kd);
    
    /*
     * Run the actual controller. Make sure this gets called reasonably quickly (i.e. every 20 ms or so)
     */
    void runLoop(bool print);

    /*
    * Get the current measured motor velocity in rad/s
    */
    float getCurrentVelocity();

    /*
    * Get number of counts from encoder - for debugging
    */
    long getCounts();

  private:

    // Pins
    int pwmPin_;
    int dirPin_;   

    // Values for computation
    double inputVel_;              // Desired velocity in rad/sec
    double currentVelRaw_;         // Raw current velocity in rad/sec
    double currentVelFiltered_;    // Filtered velocity in rad/sec
    double pidOut_;                // Output from PID controller
    int outputCmd_;                // Output command in [-255, 255]
    long prevCount_;               // Encoder count from previous loop

    // Timer
    unsigned long prevMicros_;   // Timer from previous loop

    // Other objects
    Encoder enc_;                  // Encoder object
    PID controller_;               // PID controller
    FilterOnePole velFilter_;      // Velocity lowpass filter
};

#endif Motor_h
