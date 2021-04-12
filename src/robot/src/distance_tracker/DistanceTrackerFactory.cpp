#include "DistanceTrackerFactory.h"
#include "DistanceTracker.h"
#include "DistanceTrackerMock.h"

#include <plog/Log.h> 

DistanceTrackerFactory* DistanceTrackerFactory::instance = NULL;


DistanceTrackerFactory* DistanceTrackerFactory::getFactoryInstance()
{
    if(!instance)
    {
        instance = new DistanceTrackerFactory;
    }
    return instance;
}

void DistanceTrackerFactory::set_mode(DISTANCE_TRACKER_FACTORY_MODE mode)
{
    mode_ = mode;
}

DistanceTrackerBase* DistanceTrackerFactory::get_distance_tracker()
{
    if (!distance_tracker_)
    {
        build_distance_tracker();
    }
    return distance_tracker_.get();
}


void DistanceTrackerFactory::build_distance_tracker()
{    
    if(mode_ == DISTANCE_TRACKER_FACTORY_MODE::STANDARD)
    {
        distance_tracker_ = std::make_unique<DistanceTracker>();
        PLOGI << "Built DistanceTracker";

    }
    else if (mode_ == DISTANCE_TRACKER_FACTORY_MODE::MOCK)
    {
        distance_tracker_ = std::make_unique<DistanceTrackerMock>();
        PLOGI << "Built DistanceTrackerMock";
    }
}


// Private constructor
DistanceTrackerFactory::DistanceTrackerFactory()
: mode_(DISTANCE_TRACKER_FACTORY_MODE::STANDARD),
  distance_tracker_()
{}
  