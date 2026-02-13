#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_node");
    ros::NodeHandle nh;

    ROS_INFO("example_node started.");

    ros::spin();
    return 0;
}
