#ifndef CameraTrackerBase_h
#define CameraTrackerBase_h

#include "utils.h"

class CameraTrackerBase
{
  public:

    virtual void processImage() = 0;

    virtual Point getPoseFromCamera() = 0;    
};


#endif //CameraTrackerBase_h