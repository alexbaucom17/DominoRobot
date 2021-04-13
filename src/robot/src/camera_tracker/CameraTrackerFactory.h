#ifndef CameraTrackerFactory_h
#define CameraTrackerFactory_h

#include <memory>
#include <map>

#include "CameraTrackerBase.h"

enum class CAMERA_TRACKER_FACTORY_MODE
{
  STANDARD,
  MOCK,
};

class CameraTrackerFactory
{

  public:

    static CameraTrackerFactory* getFactoryInstance();

    void set_mode(CAMERA_TRACKER_FACTORY_MODE mode);

    CameraTrackerBase* get_camera_tracker();

    // Delete copy and assignment constructors
    CameraTrackerFactory(CameraTrackerFactory const&) = delete;
    CameraTrackerFactory& operator= (CameraTrackerFactory const&) = delete;

  private:

    // Make standard constructor private so it can't be created
    CameraTrackerFactory();

    void build_camera_tracker();

    static CameraTrackerFactory* instance;

    CAMERA_TRACKER_FACTORY_MODE mode_;

    std::unique_ptr<CameraTrackerBase> camera_tracker_;

};


#endif //CameraTrackerFactory_h