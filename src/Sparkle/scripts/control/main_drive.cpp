#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_drive");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    std_msgs::String msg;
    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}