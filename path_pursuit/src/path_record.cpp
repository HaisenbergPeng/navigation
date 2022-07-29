#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int invertal_;
std::string filename;

void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
    static int count=0;
    ROS_INFO("count is %d", count);
    if(count<invertal_)
	{
        std::ofstream dataFile;
        dataFile.open(filename, std::ios::out | std::ios::app);
        // 朝TXT文档中写入数据
        dataFile << pose_msg.pose.pose.position.x <<' ' << pose_msg.pose.pose.position.y << std::endl;
        // 关闭文档
        dataFile.close();
        count++;
    }
    else
        count = 0;
}

int main(int argc, char **argv) 
{ 
//Initiate ROS  
 ros::init(argc, argv, "path_record"); 
 ros::NodeHandle n; 
 ros::NodeHandle private_nh("~"); 
 private_nh.param("/path_record/invertal", invertal_, 1);
 private_nh.param("/path_record/filename", filename, std::string("/home/robot/catkin_ws/src/path_pursuit/path/path.txt"));
 ROS_INFO("invertal is %d", invertal_);

 ros::Subscriber sub_am = n.subscribe("/amcl_pose1", 1, &AmclCallback);
 ros::spin();
 return 0; 
} 

