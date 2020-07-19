#ifndef TrayController_h
#define TrayController_h

#include "spdlog/spdlog.h"

class TrayController
{
  public:

    TrayController();

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

    ACTION curAction_;
    uint8_t actionStep_;
    bool loadComplete_;
    unsigned long startMillisForTimer_;
    spdlog::logger* logger;

    void updateInitialize();
    void updateLoad();
    void updatePlace();

};

#endif
