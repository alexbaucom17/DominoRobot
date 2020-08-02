#ifndef TrayController_h
#define TrayController_h

#include "serial/SerialComms.h"

class TrayController
{
  public:

    TrayController();

    void initialize();

    void place();

    void load();

    bool isActionRunning() {return action_running_;}

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

    const std::map<ACTION, std::string> EXPECTED_MSG = {
        {ACTION::NONE, "none"}, 
        {ACTION::INITIALIZE, "homing"}, 
        {ACTION::PLACE, "pos"}, 
        {ACTION::LOAD, "pos"}
    };

    std::unique_ptr<SerialCommsBase> serial_to_lifter_driver_;   // Serial connection to lifter driver
    bool action_running_;
    bool loadComplete_;
    ACTION cur_action_;


};

#endif