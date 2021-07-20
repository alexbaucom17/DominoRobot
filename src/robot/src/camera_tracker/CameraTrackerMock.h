#ifndef CameraTrackerMock_h
#define CameraTrackerMock_h

#include "utils.h"

class CameraTrackerMock : public CameraTrackerBase
{
  public:

    virtual void start() override {};

    virtual void stop() override {};

    virtual void update() override {};

    virtual bool running() override {return true;};

    virtual void toggleDebugImageOutput() override {};

    virtual CameraTrackerOutput getPoseFromCamera() override { return {{0,0,0},false, ClockFactory::getFactoryInstance()->get_clock()->now(),false};};

    virtual CameraDebug getCameraDebug() override {return CameraDebug(); }; 
};


#endif //CameraTrackerMock_h