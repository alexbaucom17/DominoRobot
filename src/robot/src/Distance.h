#ifndef Distance_h
#define Distance_h

// #include <thread>
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

    // Get latest distance values
    float getDistance();

    void checkForMeasurement();
  private:

    bool isRunning();
    float getMeasurement();

    float current_distance_mm_;
    CircularBuffer<float> distance_buffer_;
    bool running_;

    // std::thread run_thread_;
    SerialCommsBase* serial_to_arduino_; 
};


#endif //Distance_h