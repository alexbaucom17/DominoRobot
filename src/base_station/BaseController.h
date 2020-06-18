#ifndef BaseController_h
#define BaseController_h

#include <HardwareSerial.h>
#include "StatusUpdater.h"

class BaseController
{
  public:

    // Constructor
    BaseController(StatusUpdater& statusUpdater, HardwareSerial& debug);

    void update();
    void load();
    void estop();

  private:

    StatusUpdater& statusUpdater_;
    HardwareSerial& debug_;
    
    /* Phases
    * 0 = stopped
    * 1 = forward
    * 2 = pause
    * 3 = backward
    */
    int load_phase_;
    unsigned long load_timer_;

};


#endif