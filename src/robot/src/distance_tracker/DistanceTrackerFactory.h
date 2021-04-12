#ifndef DistanceTrackerFactory_h
#define DistanceTrackerFactory_h

#include <memory>
#include <map>

#include "DistanceTrackerBase.h"

enum class DISTANCE_TRACKER_FACTORY_MODE
{
  STANDARD,
  MOCK,
};

class DistanceTrackerFactory
{

  public:

    static DistanceTrackerFactory* getFactoryInstance();

    void set_mode(DISTANCE_TRACKER_FACTORY_MODE mode);

    DistanceTrackerBase* get_distance_tracker();

    // Delete copy and assignment constructors
    DistanceTrackerFactory(DistanceTrackerFactory const&) = delete;
    DistanceTrackerFactory& operator= (DistanceTrackerFactory const&) = delete;

  private:

    // Make standard constructor private so it can't be created
    DistanceTrackerFactory();

    void build_distance_tracker();

    static DistanceTrackerFactory* instance;

    DISTANCE_TRACKER_FACTORY_MODE mode_;

    std::unique_ptr<DistanceTrackerBase> distance_tracker_;

};


#endif //DistanceTrackerFactory_h