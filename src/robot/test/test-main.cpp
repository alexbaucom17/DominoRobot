#define CATCH_CONFIG_RUNNER
#include <Catch/catch.hpp>
#include "constants.h"
#include "serial/SerialCommsFactory.h"
#include "sockets/SocketMultiThreadWrapperFactory.h"
#include "distance_tracker/DistanceTrackerFactory.h"
#include "camera_tracker/CameraTrackerFactory.h"
#include "utils.h"

#include <plog/Log.h> 
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Formatters/MessageOnlyFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Appenders/RollingFileAppender.h>

libconfig::Config cfg = libconfig::Config();

void configure_logger()
{
    // Make file names
    std::string test_log_file_name = std::string("log/test_log.txt");

    // Initialize test logs to to go file and console
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender(test_log_file_name.c_str(), 1000000, 5);
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::info, &fileAppender).addAppender(&consoleAppender); 

    PLOGI << "Logger ready";
}

int main( int argc, char* argv[] ) 
{
    cfg.readFile(TEST_CONSTANTS_FILE);
    configure_logger();
    SerialCommsFactory::getFactoryInstance()->set_mode(SERIAL_FACTORY_MODE::MOCK);
    SocketMultiThreadWrapperFactory::getFactoryInstance()->set_mode(SOCKET_FACTORY_MODE::MOCK);
    ClockFactory::getFactoryInstance()->set_mode(CLOCK_FACTORY_MODE::MOCK);
    DistanceTrackerFactory::getFactoryInstance()->set_mode(DISTANCE_TRACKER_FACTORY_MODE::MOCK);
    CameraTrackerFactory::getFactoryInstance()->set_mode(CAMERA_TRACKER_FACTORY_MODE::MOCK);

    int result = Catch::Session().run(argc, argv);

    return result;
}