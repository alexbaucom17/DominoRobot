#ifndef CameraTrackerMock_h
#define CameraTrackerMock_h

#include "utils.h"

class CameraTrackerMock : public CameraTrackerBase
{
  public:

    virtual void processImages() override {};

    virtual Point getPoseFromCamera() override {return {0,0,0}; };  
};


#endif //CameraTrackerMock_h