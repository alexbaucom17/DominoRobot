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

    bool isActionRunning() {return cur_action_ != ACTION::NONE;}

    void update();

    void estop();

    void setLoadComplete() {load_complete_ = true;};

  private:

    enum ACTION
    {
        NONE,
        INITIALIZE,
        PLACE,
        LOAD,
    };

    std::unique_ptr<SerialCommsBase> serial_to_lifter_driver_;   // Serial connection to lifter driver
    bool action_step_running_;
    bool load_complete_;
    int action_step_;
    ACTION cur_action_;

    void runStepAndWaitForCompletion(std::string data, std::string debug_print);
    void updateInitialize();
    void updateLoad();
    void updatePlace();


};

#endif