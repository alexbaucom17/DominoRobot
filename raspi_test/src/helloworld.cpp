#include "hello.h"
#include <iostream>
#include <Eigen/Dense>
#include "ArduinoJson/ArduinoJson.h"
#include <plog/Log.h> 
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ConsoleAppender.h>
#include <plog/Appenders/RollingFileAppender.h>

void log_test()
{
    static plog::RollingFileAppender<plog::TxtFormatter> fileAppender("Hello.txt", 8000, 3); // Create the 1st appender.
    static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    // Step3: write log messages using a special macro
    // There are several log macros, use the macro you liked the most

    PLOGD << "Hello log!"; // short macro
    PLOG_DEBUG << "Hello log!"; // long macro
    PLOG(plog::debug) << "Hello log!"; // function-style macro
    
    // Also you can use LOG_XXX macro but it may clash with other logging libraries
    LOGD << "Hello log!"; // short macro
    LOG_DEBUG << "Hello log!"; // long macro
    LOG(plog::debug) << "Hello log!"; // function-style macro
}

void eigen_test()
{
    Eigen::MatrixXd m = Eigen::MatrixXd::Random(3,3);
    m = (m + Eigen::MatrixXd::Constant(3,3,1.2)) * 50;
    std::cout << "m =" << std::endl << m << std::endl;
    Eigen::VectorXd v(3);
    v << 1, 2, 3;
    std::cout << "m * v =" << std::endl << m * v << std::endl;
}

void json_test()
{
    char json[] ="{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) 
    {
        std::cout<<"deserializeJson() failed: "<<error.c_str()<<std::endl;
        return;
    }

    const char* sensor = doc["sensor"];
    long time = doc["time"];
    double latitude = doc["data"][0];
    double longitude = doc["data"][1];

    // Print values.
    std::cout<<sensor<<std::endl;
    std::cout<<time<<std::endl;
    std::cout<<latitude<<std::endl;
    std::cout<<longitude<<std::endl;
}

int main()
{
    printHello();
    log_test();
    eigen_test();
    json_test();
    std::cout<<"Bye"<<std::endl;

    return(0);
}