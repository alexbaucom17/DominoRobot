#include <plog/Log.h> 
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ConsoleAppender.h>
#include <plog/Appenders/RollingFileAppender.h>

#include "robot.h"


void configure_logger()
{
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender("log/robot_log.txt", 8000, 10); // Create the 1st appender.
    static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    PLOGI.printf("Logger ready");
}

int main()
{
    configure_logger();

    Robot r;
    r.run(); //Should loop forver until stopped

    return 0;
}
