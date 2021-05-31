#ifndef TrayController_h
#define TrayController_h

#include "serial/SerialComms.h"
#include "utils.h"

class TrayController
{
  public:

    TrayController();

    void initialize();

    // Returns bool indicating if action started successfully
    bool place();

    // Returns bool indicating if action started successfully
    bool load();

    bool isActionRunning() {return cur_action_ != ACTION::NONE;}

    void update();

    void estop();

    void setLoadComplete();

    // Used for testing
    void setTrayInitialized(bool value) {is_initialized_ = value;};

  private:

    enum ACTION
    {
        NONE,
        INITIALIZE,
        PLACE,
        LOAD,
    };

    SerialCommsBase* serial_to_lifter_driver_;   // Serial connection to lifter driver
    bool action_step_running_;
    bool load_complete_;
    int action_step_;
    bool fake_tray_motion_;
    ACTION cur_action_;
    RateController controller_rate_;       // Rate limit controller loop
    Timer action_timer_;
    bool is_initialized_;

    void runStepAndWaitForCompletion(std::string data, std::string debug_print);
    void updateInitialize();
    void updateLoad();
    void updatePlace();


};

#endif