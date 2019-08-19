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
    fd = serial_open_file("/dev/ttyUSB9", 57600);
    std::cout << "serial port " << fd << " opened!" << std::endl;

	char* info = (char *)malloc(512);
	int size = 0;
	while (ros::ok()) {
		int msg_num = receive_message(fd, info, &size);
		std::cout << "msg_num=" << msg_num << std::endl;
		// char *msg_v = (char *)malloc(size);
		// memcpy(msg_v, info, size);

		if(msg_num > 0)
		{
			int counter = 0;
			int last_counter = 0;
			for(int i=0; i<msg_num; i++)
			{
				while(counter < size)
				{
					if(*(info+counter) == '$')
					{
						/*Get level*/
						unsigned int info_level = (unsigned int)*(info+last_counter);
						// std::cout << "info_level=" << info_level <<std::endl;
						/*Get this msg*/
						int this_msg_size = counter - last_counter -1;
						char *msg_this = (char *)malloc(this_msg_size);
						memcpy(msg_this, info+last_counter+1, this_msg_size); // get data

						last_counter = counter+1; // +1 to remove simbol '\n'
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

					counter ++;
				}
				if(counter >= size) break;
			}
		}

		ros::spinOnce();
		sleep(0.1);
	}
	return 0;
}
