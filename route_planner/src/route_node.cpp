#include <iostream>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <route_planner/Astar.h>

using namespace std;
// using point_t = std::vector< double >;
// using pointVec = std::vector< point_t >;

// point_t pt(2);

Point start(0.0,0.0);
Point target(0.0,0.0);
bool plan_flag = 0;
bool get_goal = 0;
bool get_start = 0;

void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
  ROS_INFO("Getting goal");
  target.x = goal->pose.position.x;
  target.y = goal->pose.position.y;
  get_goal = 1;
  plan_flag = 1;
}

void poseCallback(const nav_msgs::Odometry &start_pose)
{
  if(plan_flag == 1 && get_start == 0){ // 这里表示只接受一次start point
    ROS_INFO("Getting start");
    start.x = start_pose.pose.pose.position.x;
    start.y = start_pose.pose.pose.position.y;
    cout<<"start = "<<start.x<<" "<<start.y<<endl;
    get_start = 1;
  }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "route_planner");
    ros::NodeHandle nh;
    string pose_topic;
    string data;
    double rad;
    nh.getParam("/route_planner/filename",data);
    nh.getParam("/route_planner/rad",rad);
    nh.getParam("/route_planner/pose_topic",pose_topic);
    ros::Subscriber sub1 = nh.subscribe("/goal",1,goalCallback);
    ros::Subscriber sub2 = nh.subscribe(pose_topic.c_str(),1,poseCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/route_plan",1000);


    ROS_INFO("string_param_init: %s", data.c_str());
    ROS_INFO("int_param_init: %f", rad);

    pointVec points;
    ifstream in;
    in.open(data);
    while(!in.eof())
    {
        double time,x,y,z,roll,pitch,yaw;
	      in>>time>>x>>y>>z>>roll>>pitch>>yaw;
        points.push_back({x,y});
    }

    int total_points = points.size();
    ROS_INFO("Get total points = %d", total_points);
    ROS_INFO("Generating success");
    in.close();

    Astar a(rad,points);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
      //ROS_INFO("plan_flag = %d",plan_flag);
      //ROS_INFO("get_goal = %d",get_goal);
      //ROS_INFO("get_start = %d",get_start);
      if(plan_flag == 1 && get_goal == 1 && get_start == 1)
      {
        ROS_INFO("Getting plan");
        point_t start_raw = {start.x,start.y};
        point_t start_nearest;

        start_nearest = a.waypoints.nearest_point(start_raw); // kdtree

        start.x = start_nearest[0];
        start.y = start_nearest[1];
        point_t goal_raw = {target.x,target.y};
        point_t goal_nearest;

        goal_nearest = a.waypoints.nearest_point(goal_raw);

        target.x = goal_nearest[0];
        target.y = goal_nearest[1];
        cout<<target.x<<"   "<<target.y<<endl;

        nav_msgs::Path planning;
        list<Point *> path = a.GetPath(start, target, false);
        for (auto &p : path)
        {
          geometry_msgs::PoseStamped pose_convert;
          pose_convert.header.frame_id = "map";
          pose_convert.pose.position.x = p->x;
          pose_convert.pose.position.y = p->y;
          planning.poses.push_back(pose_convert);
        }
        planning.header.frame_id = "map";
        path_pub.publish(planning);
        plan_flag = 0;
        get_goal = 0;
        get_start = 0;
      }
      ros::spinOnce();//
      loop_rate.sleep();
    }
    return 0;
}
