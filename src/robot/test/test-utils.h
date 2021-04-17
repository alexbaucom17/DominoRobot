#ifndef testutils_h
#define testutils_h

#include "serial/MockSerialComms.h"
#include "serial/SerialCommsFactory.h"
#include "constants.h"
#include "sockets/SocketMultiThreadWrapperFactory.h"
#include "sockets/MockSocketMultiThreadWrapper.h"
#include "distance_tracker/DistanceTrackerFactory.h" 
#include "distance_tracker/DistanceTrackerMock.h" 
#include "utils.h"
#include <variant>

inline MockSocketMultiThreadWrapper* build_and_get_mock_socket() 
{
    SocketMultiThreadWrapperBase* base_socket = SocketMultiThreadWrapperFactory::getFactoryInstance()->get_socket();
    // Slightly dangerous....
    MockSocketMultiThreadWrapper* mock_socket = dynamic_cast<MockSocketMultiThreadWrapper*>(base_socket);
    mock_socket->purge_data();
    return mock_socket;
}

inline MockSerialComms* build_and_get_mock_serial(const std::string& portName)
{
    SerialCommsBase* base_serial = SerialCommsFactory::getFactoryInstance()->get_serial_comms(portName);
    // Slightly dangerous....
    MockSerialComms* mock_serial = dynamic_cast<MockSerialComms*>(base_serial);
    mock_serial->purge_data();
    return mock_serial;
}

inline DistanceTrackerMock* build_and_get_mock_distance_tracker() 
{
    DistanceTrackerBase* base_distance_tracker = DistanceTrackerFactory::getFactoryInstance()->get_distance_tracker();
    // Slightly dangerous....
    DistanceTrackerMock* mock_distance_tracker = dynamic_cast<DistanceTrackerMock*>(base_distance_tracker);
    return mock_distance_tracker;
}

inline MockClockWrapper* get_mock_clock()
{
    ClockWrapperBase* base_clock = ClockFactory::getFactoryInstance()->get_clock();
    MockClockWrapper* mock_clock = dynamic_cast<MockClockWrapper*>(base_clock);
    return mock_clock;
}

inline MockClockWrapper* get_mock_clock_and_reset()
{
    MockClockWrapper* mock_clock = get_mock_clock();
    mock_clock->set_now();
    return mock_clock;
}

inline void reset_mock_clock()
{
    MockClockWrapper* mock_clock = get_mock_clock();
    mock_clock->set_now();
}

// Helper class to modify a global setting which will revert the setting change when going out of scope
template <typename T>
class SafeConfigModifier
{
  public:
    SafeConfigModifier(std::string config_path, T value)
     : cur_val_(cfg.lookup(config_path)),
       old_val_(static_cast<T>(cur_val_))
    {
        cur_val_ = value;
    }
    ~SafeConfigModifier()
    {
        cur_val_ = old_val_;
    }

  private:
    libconfig::Setting& cur_val_;
    T old_val_;
};


// class SafeConfigModifierContainer
// {
//   public:
//     template <typename T>
//     void add(std::string path, T value)
//     {
//         data_.push_back(SafeConfigModifier<T>(path, value));
//     }
//   private:
//     std::vector<std::variant<
//       SafeConfigModifier<int>,
//       SafeConfigModifier<float>,
//       SafeConfigModifier<std::string>,
//       SafeConfigModifier<bool>
//       >> data_;
// };

#endif