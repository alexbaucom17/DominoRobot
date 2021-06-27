#include <plog/Log.h> 
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Formatters/MessageOnlyFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Appenders/RollingFileAppender.h>
#include <chrono>
#include <iostream>

#include "robot.h"
#include "constants.h"
#include "sockets/SocketMultiThreadWrapperFactory.h"
#include "camera_tracker/CameraTrackerFactory.h"

libconfig::Config cfg = libconfig::Config();

void configure_logger()
{
    // Get current date/time
    const std::time_t datetime =  std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char datetime_str[50];
    std::strftime(datetime_str, sizeof(datetime_str), "%Y%m%d_%H%M%S", std::localtime(&datetime));

    // Make file names
    std::string robot_log_file_name = std::string("log/robot_log_") + std::string(datetime_str) + std::string(".txt");
    std::string motion_log_file_name = std::string("log/motion_log_") + std::string(datetime_str) + std::string(".txt");
    std::string localization_log_file_name = std::string("log/localization_log_") + std::string(datetime_str) + std::string(".txt");

    // Initialize robot logs to to go file and console
    std::string log_level = cfg.lookup("log_level");
    plog::Severity severity = plog::severityFromString(log_level.c_str());
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender(robot_log_file_name.c_str(), 1000000, 5);
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(severity, &fileAppender).addAppender(&consoleAppender); 

    // Initialize motion logs to go to file
    static plog::RollingFileAppender<plog::TxtFormatter> motionFileAppender(motion_log_file_name.c_str(), 1000000, 5);
    plog::init<MOTION_LOG_ID>(plog::debug, &motionFileAppender);

    // Initialize localization logs to go to file
    static plog::RollingFileAppender<plog::TxtFormatter> localizationFileAppender(localization_log_file_name.c_str(), 1000000, 5);
    plog::init<LOCALIZATION_LOG_ID>(plog::debug, &localizationFileAppender);

    PLOGI << "Logger ready";
}

void setup_mock_socket()
{
    bool enabled = cfg.lookup("mock_socket.enabled");
    if(enabled)
    {
        auto factory = SocketMultiThreadWrapperFactory::getFactoryInstance();
        factory->set_mode(SOCKET_FACTORY_MODE::MOCK);
        factory->build_socket();
        for (const auto& s : cfg.lookup("mock_socket.data"))
        {
            factory->add_mock_data(s);
        }
    }
}

void capture_images()
{
    // Modify config values to ensure they are set to log debug images
    libconfig::Setting& save_debug_config = cfg.lookup("vision_tracker.debug.save_camera_debug");
    save_debug_config = true;
    
    // Start camera
    CameraTrackerBase* camera_tracker_ = CameraTrackerFactory::getFactoryInstance()->get_camera_tracker();
    camera_tracker_->start();
    
    // Wait for a few seconds for images to save
    Timer t;
    while(t.dt_s() < 5.0) {}

    PLOGI << "Camera image capture complete...Exiting";
}

int main(int argc, char** argv)
{
    try
    {
        cfg.readFile(CONSTANTS_FILE);
        std::string name = cfg.lookup("name");

        configure_logger();
        PLOGI << "Loaded constants file: " << name;

        if(argc > 1 && std::string(argv[1]) == "-c")
        {
            capture_images();
        } 
        else 
        {
            setup_mock_socket();

            Robot r;
            r.run(); //Should loop forver until stopped
        }

    }
    catch (const libconfig::SettingNotFoundException &e)
    {
        std::cerr << "Configuration error with " << e.getPath() << std::endl;
        return(EXIT_FAILURE);
    }

    return(EXIT_SUCCESS);
}
