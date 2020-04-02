#ifndef TrayController_h
#define TrayController_h

#include <HardwareSerial.h>

class TrayController
{
  public:

    TrayController(HardwareSerial& debug);

    void initialize();

    void place();

    void load();

    bool isActionRunning() { return actionRunning_; };

  private:

    HardwareSerial& debug_;
    bool actionRunning_;

};

#endif