#include "veta/util/LogHandler.h"
#include <iostream>

#include "veta_noetic/util/Timer.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "outputhandler_demo_node");
    ros::NodeHandle nh;

    // Terminal log test
    std::cout << "Terminal log test" << std::endl;
    TIMER_START;

    veta::log::setLogLevel(veta::log::LogLevel::DEBUG);
    VETA_DEBUG("Test: %d" , 1  );
    VETA_INFO ("Test: %lf", 1.0);
    VETA_WARN ("Test: %c",  'c');
    VETA_ERROR("Test: %s", std::string("String").c_str());

    TIMER_END;
    std::cout << "---" << std::endl;

    // File log test
    std::cout << "File log test" << std::endl;
    TIMER_START;
    
    veta::log::enableLogFileHandler("/home/veta_ws/src/veta_ros/demo/util/config/log_test.txt");
    veta::log::setLogHandlerType(veta::log::HandlerType::FILE);

    VETA_DEBUG("Test: %d" , 1  );
    VETA_INFO ("Test: %lf", 1.0);
    VETA_WARN ("Test: %c",  'c');
    VETA_ERROR("Test: %s", std::string("String").c_str());

    veta::log::disableLogFileHandler();
    veta::log::setLogHandlerType(veta::log::HandlerType::TERMINAL);

    TIMER_END;
    std::cout << "---" << std::endl;

    // Terminal log test
    std::cout << "Terminal log test" << std::endl;
    TIMER_START;

    veta::log::setLogLevel(veta::log::LogLevel::DEBUG);
    VETA_DEBUG("Test: %d" , 1  );
    VETA_INFO ("Test: %lf", 1.0);
    VETA_WARN ("Test: %c",  'c');
    VETA_ERROR("Test: %s", std::string("String").c_str());

    TIMER_END;
    std::cout << "---" << std::endl;
    
    return 0;
}