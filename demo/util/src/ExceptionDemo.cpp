#include "veta/util/Exception.h"
#include "veta/util/LogHandler.h"

#include "veta_noetic/util/Timer.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "outputhandler_demo_node");
    ros::NodeHandle nh;

    TIMER_START;
    try
    {
        throw veta::Exception("ExceptionDemo.cpp", "example error type");
    }
    catch (const veta::Exception &ex)
    {
        VETA_ERROR("%s", ex.what());
    }
    TIMER_END;

    TIMER_START;
    try
    {
        throw veta::Exception("example error type");
    }
    catch (const veta::Exception &ex)
    {
        VETA_ERROR("%s", ex.what());
    }
    TIMER_END;

    return 0;
}