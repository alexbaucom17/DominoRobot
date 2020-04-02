#ifndef TrayController_h
#define TrayController_h

#include <HardwareSerial.h>
#include <Servo.h>

class TrayController
{
  public:

    TrayController(HardwareSerial& debug);

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
    ACTION curAction_;
    uint8_t actionStep_;
    bool loadComplete_;

    void updateInitialize();
    void updateLoad();
    void updatePlace();

};

#endif