#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h> 
#include <sstream>
#include "sys/time.h"
#include <math.h>
#include <stdio.h>
#include "serial.h"
#include <std_msgs/Float32.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "xbee_receive_info");
	ros::NodeHandle n;

	ros::Rate loop_rate(20);

	int fd;
    fd = serial_open_file("/dev/ttyUSB0", 57600);
    std::cout << "serial port " << fd << " opened!" << std::endl;

	char* info = (char *)malloc(512);
	int size = 0;
	while (ros::ok()) {
		int msg_num = receive_message_one_by_one(fd, info, &size);
		// std::cout << "msg_num=" << msg_num << std::endl;

		if(msg_num > 0)
		{

			/*Get level*/
			unsigned int info_level = (unsigned int)*(info);
			// std::cout << "info_level=" << info_level <<std::endl;
			/*Get this msg*/
			char *msg_this = (char *)malloc(size-1);
			memcpy(msg_this, info+1, size-1); // get data

			std::string msg_this_s = msg_this;
			//std::cout << "info_id " << info_level << " " << msg_this_s << std::endl;
			switch(info_level)
			{
				case 1:
					ROS_DEBUG_STREAM(msg_this_s);
					break;
				case 2:
					ROS_INFO_STREAM(msg_this_s);
					break;
				case 4:
					ROS_WARN_STREAM(msg_this_s);
					break;
				case 8:
					ROS_ERROR_STREAM(msg_this_s);
					break;
				case 16:
					ROS_FATAL_STREAM(msg_this_s);
					break;
				default:
					ROS_INFO_STREAM(msg_this_s);	
			}

		}

		ros::spinOnce();
		sleep(0.01);
	}
	return 0;
}
