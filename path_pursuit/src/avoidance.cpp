#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include <geometry_msgs/Twist.h>
#include <obstacle_detector/Obstacles.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
class Avoidance
{

public:

	Avoidance()
	{
		cmd_n.linear.x = cmd_n.angular.z = 0;
		obstaclestop_f = obstaclestop_b = 1;

  	sub_n = n.subscribe("/cmd_vel1", 1, &Avoidance::velCallback2, this);  //navigation cmd
  	sub_ob = n.subscribe("/obstacles", 5, &Avoidance::obstacleCallback, this);
		sub_bc = n.subscribe("/cmd_vel2", 5, &Avoidance::backCallback, this);

  	timer_ = n.createTimer(ros::Duration(0.1), boost::bind(&Avoidance::cmdpublish, this));
  	cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	}


	void velCallback2(const geometry_msgs::Twist& msg1);
        void backCallback(const geometry_msgs::Twist& msg2);
	void obstacleCallback(const obstacle_detector::Obstacles& msg);
	void cmdpublish();

private:

 ros::NodeHandle n;
 ros::Publisher cmd_vel_publisher;

 ros::Subscriber sub_n;
 ros::Subscriber sub_ob;
 ros::Subscriber sub_bc;

 char obstaclestop_f, obstaclestop_b;
 boost::mutex publish_mutex_;
 ros::Timer timer_;
 geometry_msgs::Twist cmd_n;   //navigation cmd
 geometry_msgs::Twist cmd_b;   //navigation cmd
};

int main(int argc, char **argv)
{
//Initiate ROS
 ros::init(argc, argv, "avoidance");
//Create an object of class SubscribeAndPublish that will take care of everything
 Avoidance test;

 ros::spin();
 return 0;
}


void Avoidance::velCallback2(const geometry_msgs::Twist& msg1)
{
	cmd_n = msg1;
}

void Avoidance::backCallback(const geometry_msgs::Twist& msg2)
{
	cmd_b = msg2;
}

void Avoidance::obstacleCallback(const obstacle_detector::Obstacles& msg)
{
   int n;
   int m;
   int i;
   float obdist, obstacledis=0xff;
   float r;
   float x,y;
   int flag=1;
   int flag1=1;
   n= msg.segments.size();
   m= msg.circles.size();
   for(i=0;i<m;i++)
   {
        r= msg.circles[i].radius;
        x=msg.circles[i].center.x;
        y=msg.circles[i].center.y;

      //  ROS_INFO("x=%f,y=%f",x,y);
        if(x<0.65 && x>=0 && (fabs(y+r)<0.2 || fabs(y-r)<0.2 || fabs(y)<0.2))
        {
            flag=0;
        }

        if(x>-0.7 && x<0 && (fabs(y+r)<0.2 || fabs(y-r)<0.2 || fabs(y)<0.2))
        {
            flag1=0;
        }
        //ROS_INFO("flag=%d",flag);
   }

   for(i=0;i<n;i++)
   {
        obdist=msg.segments[i].first_point.y*msg.segments[i].last_point.y;
       // x=(msg.segments[i].first_point.x+msg.segments[i].last_point.x)/2.0;

        if(obdist<=0)
        {

           obstacledis = (msg.segments[i].first_point.x+msg.segments[i].last_point.x)/2;
           break;

        }
         // }

    }



    if(flag==0 || (obstacledis<0.65 && obstacledis>=0))
    {
        this->obstaclestop_f=0;
        //ROS_INFO("obstacledis");

     }
     else
     {
        this->obstaclestop_f=1;
        //ROS_INFO("obstaclestop=1");

     }

     if(flag1==0 || (obstacledis>-0.2 && obstacledis<0))
    {
        this->obstaclestop_b=0;
        //ROS_INFO("obstaclestop=0");

     }
     else
     {
        this->obstaclestop_b=1;
        //ROS_INFO("obstaclestop=1");

     }

}

void Avoidance::cmdpublish()
{

  boost::mutex::scoped_lock lock(publish_mutex_);
  geometry_msgs::Twist cmd;

  //ROS_INFO("obstaclestop_f=%d",this->obstaclestop_f);
	if(cmd_b.linear.x<0)
	{
		cmd = cmd_b;
		if(!this->obstaclestop_b){cmd.linear.x=0.0;}
	}
	else
	{
		cmd = cmd_n;
    if(!this->obstaclestop_f)
	  {
      if(cmd.linear.x>0)
      {
  	    ROS_INFO("stop");
        cmd.linear.x=0;
  	    cmd.angular.z=0;
      }
    }

    if(!this->obstaclestop_b)
	  {
  	  if(cmd.linear.x<0)
  	  {
  		  cmd.linear.x=0;
  		  cmd.angular.z=0;
  	  }
    }
  }
  cmd_vel_publisher.publish(cmd);
}
