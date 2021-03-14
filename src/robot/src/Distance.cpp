#include "Distance.h"
#include "constants.h"
#include <pigpiod_if2.h>
#include <mutex>
#include "utils.h"
#include <plog/Log.h> 

std::mutex mutex;
#define HIGH 1
#define LOW 0

Distance::Distance()
: current_distance_mm_(0.0),
  distance_buffer_(20),
  running_(false),
  trigger_pin_(cfg.lookup("distance.trigger_pin")),
  echo_pin_(cfg.lookup("distance.echo_pin"))
{
    gpio_id_ = pigpio_start(nullptr, nullptr);
    if(gpio_id_ < 0)
    {
        PLOGE << "pigpio initialization failed";
        throw "pigpio initialization failed";
    }
    set_mode(gpio_id_, trigger_pin_, PI_OUTPUT);
    set_mode(gpio_id_, echo_pin_, PI_INPUT);

    run_thread_ = std::thread(&Distance::measurementLoop, this);
    run_thread_.detach();
}

Distance::~Distance()
{
    pigpio_stop(gpio_id_);
}

void Distance::start()
{
    std::lock_guard<std::mutex> lock(mutex);
    running_ = true;
}
void Distance::stop()
{
    std::lock_guard<std::mutex> lock(mutex);
    running_ = false;
}

float Distance::getDistance()
{
    std::lock_guard<std::mutex> lock(mutex);
    return current_distance_mm_;
}

bool Distance::isRunning() 
{
    std::lock_guard<std::mutex> lock(mutex);
    return running_;
}

void Distance::measurementLoop()
{
    while(true)
    {
        while(isRunning())
        {
            float new_measurement = doMeasurement();
            if (new_measurement < 0) continue;
            distance_buffer_.insert(new_measurement);
            const std::vector<float> data = distance_buffer_.get_contents();
            float new_mean = vectorMean(data);
            std::lock_guard<std::mutex> lock(mutex);
            current_distance_mm_ = new_mean;
        }

    }
}


float Distance::doMeasurement()
{
    // TODO: move these to class so I don't have to re-allocate memory every time in loop
    Timer timer;
    Timer timeout_timer;
    constexpr int timeout_us = 15000;

    // Trigger by pulsing for 10 us
    gpio_write(gpio_id_,trigger_pin_, HIGH);
    while(timer.dt_us() < 10) {}
    gpio_write(gpio_id_, trigger_pin_, LOW);

    // Wait for echo and measure pulse time
    int pulse_time_us = 0;
    while(!gpio_read(gpio_id_,echo_pin_)) 
    {
        timer.reset();
        if(timeout_timer.dt_us() > timeout_us)
        {
            return -1.0;
        }
    }
    while(gpio_read(gpio_id_, echo_pin_))
    {
        pulse_time_us = timer.dt_us();
        if(timeout_timer.dt_us() > timeout_us)
        {
            return -1.0;
        }
    }
    
    // Compute distance - 0.343 is speed of sound in millimeters per microsecond
    // Divide by 2 is to compensate for there and back.
    float distance = (0.343 * pulse_time_us) / 2.0;
    return distance;
}