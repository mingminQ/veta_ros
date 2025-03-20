#include "veta_noetic/transform/TransformLink.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "TransformDemo");
    ros::NodeHandle nh;

    veta::noetic::TransformLink baseLink("map", "base_link");
    baseLink.setOrigin(0.0, 3.0, 0.0, 0.0, 0.0, M_PI / 2.0);

    veta::noetic::TransformLink joint1("base_link", "lidar");
    joint1.setOrigin(0.0, 0.5, 0.0, 0.0, 0.0, 0.0);

    ros::Duration delayTime(0.25);
    while(ros::ok())
    {
        baseLink.broadcast();
        joint1.broadcast();
        delayTime.sleep();
    }

    ros::shutdown();
    return 0;
}