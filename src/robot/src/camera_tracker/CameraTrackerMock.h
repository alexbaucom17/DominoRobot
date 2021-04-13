#ifndef CameraTrackerMock_h
#define CameraTrackerMock_h

#include "utils.h"

class CameraTrackerMock : public CameraTrackerBase
{
  public:

    virtual void processImage() override {};

    virtual Point getPoseFromCamera() override {return {0,0,0}; };  
};


#endif //CameraTrackerMock_h