#ifndef TrayController_h
#define TrayController_h

#include <HardwareSerial.h>
#include <Servo.h>
#include "AccelStepper.h"

class TrayController
{
  public:

    TrayController(HardwareSerial& debug);

    void begin();

    void initialize();

    void place();

    void load();

    bool isActionRunning();

    void update();

    void estop();

    void setLoadComplete() {loadComplete_ = true;};

  private:

    enum ACTION
    {
        NONE,
        INITIALIZE,
        PLACE,
        LOAD,
    };

    HardwareSerial& debug_;
    Servo latchServo_;
    AccelStepper lifterLeft_;
    AccelStepper lifterRight_;
    ACTION curAction_;
    uint8_t actionStep_;
    bool loadComplete_;
    unsigned long startMillisForTimer_;

    void updateInitialize();
    void updateLoad();
    void updatePlace();

};

#endif
