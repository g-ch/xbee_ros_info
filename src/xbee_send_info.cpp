#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sys/time.h"
#include <math.h>
#include <stdio.h>
#include "serial.h"
#include <rosgraph_msgs/Log.h>
#include <string.h>

int initsecs;
int fd;
int flag;

using namespace std;

void rosOutCallback(const rosgraph_msgs::Log &msg)
{
    char level = (char) msg.level;
    string level_s(1,level);
    string node_name = msg.name;
    string info = msg.msg;

    string content =  level_s + '[' + node_name + ']' + info;

    /**Max msg size: 249 bytes**/
    if(content.size() > 249)
    {
        std::cout<<"This Info is larger than 200 bytes, will not send!"<<endl;
        return;
    }

    /**Note 28 bytes each time**/
    const char* data_send = content.data();

    if(fd<1)
    {
        fd = serial_open_file("/dev/ttyUSB0", 57600);
    }
    send_message(fd, content.size(), data_send);
    std::cout<< "message sent" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbee_send_info");
    ros::NodeHandle n;
    fd = serial_open_file("/dev/ttyUSB1", 57600);
    ROS_INFO("Open Serial: [%d]", fd);
    initsecs = (int)(ros::Time::now().toSec());
    ros::Subscriber sub = n.subscribe("/rosout", 10, rosOutCallback); //buffer size 10
    
    // rosgraph_msgs::Log fake_msg;
    // fake_msg.level = 8;
    // fake_msg.name = "fake node";
    // fake_msg.msg = "this is a test message";

    // ros::Rate loop_rate(100);
    // while(ros::ok())
    // {
    //     rosOutCallback(fake_msg);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();

	return 0;
}
