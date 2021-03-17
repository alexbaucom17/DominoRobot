#ifndef Distance_h
#define Distance_h

#include "utils.h"
#include "serial/SerialComms.h"

class Distance
{

  public:
    Distance();

    // Start measurement loop
    void start();

    // Stop measurement loop
    void stop();

    // Main 'update' function that must be called regularly
    void checkForMeasurement();

    // Get latest distance values
    float getDistance() {return current_distance_mm_;};

    // Returns bool indicating if distance measurements are running
    bool isRunning() { return running_;};

  private:

    // Handles getting measurements from serial port and parsing into number
    float getMeasurement();

    float current_distance_mm_;
    CircularBuffer<float> distance_buffer_;
    bool running_;

    SerialCommsBase* serial_to_arduino_; 
};


#endif //Distance_h