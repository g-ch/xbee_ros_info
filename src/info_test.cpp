#include "ros/ros.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "info_test");
    ros::NodeHandle n;


    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        ROS_INFO("info 666");
        ROS_WARN("warn 777");
        ROS_ERROR("warn 888");

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}