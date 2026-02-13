#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_pub_node");
    ros::NodeHandle nh;

    ROS_INFO("mavros_pub_node started.");

    ros::spin();
    return 0;
}
