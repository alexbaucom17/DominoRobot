#ifndef TrayController_h
#define TrayController_h


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
    int actionStep_;
    bool loadComplete_;
    unsigned long startMillisForTimer_;

    void updateInitialize();
    void updateLoad();
    void updatePlace();

};

#endif
