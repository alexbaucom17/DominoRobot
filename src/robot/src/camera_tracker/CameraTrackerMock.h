#ifndef CameraTrackerMock_h
#define CameraTrackerMock_h

#include "utils.h"

class CameraTrackerMock : public CameraTrackerBase
{
  public:

    virtual void start() override {};

    virtual void stop() override {};

    virtual CameraTrackerOutput getPoseFromCamera() override { return {{0,0,0},false};};

    virtual CameraDebug getCameraDebug() override {return CameraDebug(); }; 
};


#endif //CameraTrackerMock_h