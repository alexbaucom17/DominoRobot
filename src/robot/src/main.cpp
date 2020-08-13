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

libconfig::Config cfg = libconfig::Config();

void configure_logger()
{
    // Get current date/time
    const std::time_t datetime =  std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char datetime_str[50];
    std::strftime(datetime_str, sizeof(datetime_str), "%Y%m%d-%H%M%S", std::localtime(&datetime));

    // Make file names
    std::string robot_log_file_name = std::string("log/robot_log_") + std::string(datetime_str) + std::string(".txt");
    std::string motion_log_file_name = std::string("log/motion_log_") + std::string(datetime_str) + std::string(".txt");

    // Initialize robot logs to to go file and console
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender(robot_log_file_name.c_str(), 0, 0);
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::info, &fileAppender).addAppender(&consoleAppender); 

    // Initialize motion logs to go to file
    static plog::RollingFileAppender<plog::MessageOnlyFormatter> motionFileAppender(motion_log_file_name.c_str(), 0, 0);
    plog::init<MOTION_LOG_ID>(plog::debug, &motionFileAppender);

    PLOGI << "Logger ready";
}


// TODO - move to config file
#define MOCK_SOCKET true
std::vector<std::string> MOCK_SOCKET_DATA = {"<{'type':'init'}>", "8000", "<{'type':'place'}>", "3000", "<{'type':'estop'}>"};
void setup_mock_socket()
{
    if(MOCK_SOCKET)
    {
        auto factory = SocketMultiThreadWrapperFactory::getFactoryInstance();
        factory->set_mode(SOCKET_FACTORY_MODE::MOCK);
        factory->build_socket();
        for (const auto& s : MOCK_SOCKET_DATA)
        {
            factory->add_mock_data(s);
        }
    }
}

int main()
{
    try
    {
        configure_logger();
        
        
        cfg.readFile(CONSTANTS_FILE);
        std::string name = cfg.lookup("name");
        PLOGI << "Loaded constants file: " << name;

        setup_mock_socket();

        Robot r;
        r.run(); //Should loop forver until stopped
    }
    catch (const libconfig::SettingNotFoundException &e)
    {
        std::cerr << "Configuration error with " << e.getPath() << std::endl;
        return(EXIT_FAILURE);
    }

    return(EXIT_SUCCESS);
}
