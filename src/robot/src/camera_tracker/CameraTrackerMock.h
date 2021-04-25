#ifndef CameraTrackerMock_h
#define CameraTrackerMock_h

#include "utils.h"

class CameraTrackerMock : public CameraTrackerBase
{
  public:

    virtual void start() override {};

    virtual void stop() override {};

    virtual CameraTrackerOutput getPoseFromCamera() override { return {{0,0,0},false};}; 

    virtual int getLoopTimeMs() override {return 0;};
};


#endif //CameraTrackerMock_h