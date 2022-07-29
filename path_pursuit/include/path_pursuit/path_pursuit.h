#ifndef PATHPURSUIT_H
#define PATHPURSUIT_H

#include <ros/ros.h>
#include <string>
#include <math.h> 
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread.hpp>
#include <sstream>
#include <fstream>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <vector>
#include <utility>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>



class Pursuit
{
   public: 
        Pursuit(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~Pursuit();
        void calPID();
		
   private:
        ros::NodeHandle n_; 
        ros::NodeHandle nh_private_;
        ros::Publisher cmd_vel_publisher_;
        ros::Subscriber sub1; 
        ros::Subscriber sub2; 

        double posx_;                                         //当前位姿
        double posy_;
        double the_;
        double nextx;                                         //下一个要跟踪的目标点
        double nexty;
        double lastx;                                         //上一个跟踪的目标点
        double lasty;
        double dist;                                          //当前位置与目标点间距离
        double len_th;                                        //前瞻
        std::string filename;                                 //轨迹文件名称
        std::vector<std::pair<double,double>> path;           //路径点
        int pointsize;
        int point_p;                                          //当前搜寻到到路径点
        double vel;                                           //目标线速度
        int goalflag;                                         //是否在目标点附近
        double dist_th;                                       //目标点容忍度
        int endtimes;
        double acc_err;
        double avg_err;
        int pathRead;
        std::string cmd_vel;
        std::string pose_topic;

   private://member functions
        //void posecallback(const geometry_msgs::PoseStamped& posmsg);
        void posecallback(const nav_msgs::Odometry& posmsg);
        void readpath();
        void findnextpoint();
        void initParam();
        void pathcallback(const nav_msgs::Path& pathmsg);

};


#endif
