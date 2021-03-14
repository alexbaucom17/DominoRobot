#ifndef Distance_h
#define Distance_h

#include <thread>

class Distance
{

  public:
    Distance();
    ~Distance();

    // Start measurement loop
    void start();

    // Stop measurement loop
    void stop();

    // Get latest distance values
    float getDistance();

  private:

    void measurementLoop();
    bool isRunning();
    float doMeasurement();

    float current_distance_mm_;
    bool running_;
    int trigger_pin_;
    int echo_pin_;
    int gpio_id_;

    std::thread run_thread_;
};


#endif //Distance_h