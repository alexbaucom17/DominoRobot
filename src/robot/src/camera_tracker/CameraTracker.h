#ifndef CameraTracker_h
#define CameraTracker_h

#include "CameraTrackerBase.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

class CameraTracker : public CameraTrackerBase
{
  public:
    CameraTracker();

    virtual void start() override;

    virtual void stop() override;

    virtual Point getPoseFromCamera() override; 

    void test_function();   

  private:
    cv::SimpleBlobDetector blob_detector_;
    bool running_;
    
};


#endif //CameraTracker_h