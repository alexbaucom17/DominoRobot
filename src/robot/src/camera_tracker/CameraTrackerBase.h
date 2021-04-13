#ifndef CameraTrackerBase_h
#define CameraTrackerBase_h

#include "utils.h"

class CameraTrackerBase
{
  public:

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual Point getPoseFromCamera() = 0;    
};


#endif //CameraTrackerBase_h