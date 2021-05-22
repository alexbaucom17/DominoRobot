#include "CameraTrackerFactory.h"
#include "CameraTracker.h"
#include "CameraTrackerMock.h"

#include <plog/Log.h> 

CameraTrackerFactory* CameraTrackerFactory::instance = NULL;


CameraTrackerFactory* CameraTrackerFactory::getFactoryInstance()
{
    if(!instance)
    {
        instance = new CameraTrackerFactory;
    }
    return instance;
}

void CameraTrackerFactory::set_mode(CAMERA_TRACKER_FACTORY_MODE mode)
{
    mode_ = mode;
}

CameraTrackerBase* CameraTrackerFactory::get_camera_tracker()
{
    if (!camera_tracker_)
    {
        build_camera_tracker();
    }
    return camera_tracker_.get();
}


void CameraTrackerFactory::build_camera_tracker()
{    
    if(mode_ == CAMERA_TRACKER_FACTORY_MODE::STANDARD)
    {
        camera_tracker_ = std::make_unique<CameraTracker>();
        PLOGI << "Built CameraTracker";

    }
    else if (mode_ == CAMERA_TRACKER_FACTORY_MODE::MOCK)
    {
        camera_tracker_ = std::make_unique<CameraTrackerMock>();
        PLOGI << "Built CameraTrackerMock";
    }
}


// Private constructor
CameraTrackerFactory::CameraTrackerFactory()
: mode_(CAMERA_TRACKER_FACTORY_MODE::STANDARD),
  camera_tracker_()
{}
  