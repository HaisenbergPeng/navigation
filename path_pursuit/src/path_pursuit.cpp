#include "path_pursuit/path_pursuit.h"
using namespace std; 

Pursuit::Pursuit(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	n_ = nh;
	nh_private_ = nh_private;
	initParam();
	sub1 = n_.subscribe(pose_topic, 5, &Pursuit::posecallback, this);
	//sub2 = n_.subscribe("/route_plan", 5, &Pursuit::pathcallback, this);
	cmd_vel_publisher_ = n_.advertise<geometry_msgs::Twist>(this->cmd_vel,5); 	
	// readpath();
	//ROS_INFO("The number of points is %d", pointsize);
}

Pursuit::~Pursuit()
{
	ROS_INFO("Destroying path_pursuit");
}

void Pursuit::initParam()  
{
	posx_ = 0.0;
	posy_ = 0.0;
	the_= 0.0;
	dist = 0.0;
	nextx = 0.0;
	nexty = 0.0;
	vel = 0.3;
	goalflag = 0;
	pathRead = 0;
	nh_private_.param("len_th", len_th, 0.8);
	nh_private_.param("dist_th", dist_th, 0.6);
	nh_private_.param("endtimes", endtimes, 5);
	nh_private_.param("filename", filename, std::string(""));
	nh_private_.param("cmd_vel", cmd_vel, std::string("cmd_vel"));
	nh_private_.param("pose_topic", pose_topic, std::string("/kloam/mapping/odometry"));
	
}

void Pursuit::pathcallback(const nav_msgs::Path& pathmsg)
{
	//reachflag = 0;
	//goalflag = 0;
	ROS_INFO("Geting plan");
	point_p=0;
	pointsize = pathmsg.poses.size();
	ROS_INFO("Totally we have %d waypoints",pointsize);
	path.clear();
	for (auto &p : pathmsg.poses)
	{
		double x,y;
		x = p.pose.position.x;
		y = p.pose.position.y;
		path.push_back(make_pair(x,y));
    
	}
	pathRead = 1;

}

void Pursuit::readpath()
{
	point_p=0;
	acc_err=0;
	avg_err=0;
	path.clear();
	ifstream in(filename, std::ifstream::in);
	if (in.good())
	{			
		string line;		
		while (getline(in, line))
		{
			istringstream stream(line);
			string x,y;
			stream >> x >> y;							
			path.push_back(make_pair(atof(x.c_str()),atof(y.c_str())));		
		
		}
		in.close();
		pointsize = path.size();
		pathRead = 1;
		ROS_INFO("Path reading complete");
	}
	else pathRead = 0;
}

/**找下一个目标点**/
void Pursuit::findnextpoint()
{
	//ROS_INFO("findpoint");
	if(point_p!=0)
	{
		lastx = path[point_p].first;
		lasty = path[point_p].second;
	}
	while(point_p < pointsize)
	{
		this->dist = sqrt(pow(path[point_p].first - this->posx_, 2) + pow(path[point_p].second-this->posy_, 2));		
		if(this->dist >= this->len_th) 
			break;		// continue trying to reach it
		point_p++;
		//ROS_INFO("point_p is %d", point_p);
	}
	// if(point_p!=0)
	// 	{
	// 		acc_err =acc_err + sqrt(pow(lastx- this->posx_, 2) + pow(lasty-this->posy_, 2));
	// 		avg_err = acc_err / (point_p + 1);
	// 		ROS_INFO("acc_err=%f",acc_err);
	// 		ROS_INFO("avg_err=%f",avg_err);
	// 	}
	//ROS_INFO("the %dth point", point_p);
	if(point_p < pointsize)
	{	
		nextx = path[point_p].first;
		nexty = path[point_p].second;
	}
	else
	{
		nextx = path[pointsize-1].first;
		nexty = path[pointsize-1].second;
	}
	//结束节点
	if(point_p>=pointsize)
	{
		this->dist = sqrt(pow(path.back().first-this->posx_, 2) + pow(path.back().second-this->posy_, 2));
		ROS_INFO("distance to the goal = %f",this->dist);
		if(this->dist <= dist_th) 
		{	
			goalflag++;
		}
		if(goalflag>=endtimes)
		{
			ROS_INFO("Goal reached, now shut down");
			ros::shutdown();
		}
			
	}
	return;
}

/**定位结果到回调函数，计算目标速度并发布**/
void Pursuit::posecallback(const nav_msgs::Odometry& posmsg)
{
	//ROS_INFO("posecallback");
	//ROS_INFO("goal flag = %d",goalflag);
	//ROS_INFO("dist_th = %f",dist_th);
	if (pathRead)
	{
		double roll, pitch, yaw;                               //当前amcl计算所得位姿
		this->posx_ = posmsg.pose.pose.position.x;
		this->posy_ = posmsg.pose.pose.position.y;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(posmsg.pose.pose.orientation, quat); 
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		this->the_ = yaw;
		findnextpoint();                                       //根据位姿计算下一个目标点
		calPID();                                              //计算速度输出
	}
	
}

/**根据距离角度误差计算输出**/
void Pursuit::calPID()
{
	double yy = nexty - this->posy_;
	double xx = nextx - this->posx_;
	double beta = atan2(yy, xx);                                 //机器人与目标点连线夹角
	double apha = beta - this->the_;

	geometry_msgs::Twist cmd;
	double cosa = cos(apha);
	double sina = sin(apha);

	//cmd.linear.x = 0.3*(this->dist+0.10);
	cmd.linear.x = 0.4*(this->dist+0.10);
	if (cmd.linear.x > 0.5)
		cmd.linear.x = 0.5;
	while(apha>3.14)
		apha -= 2*3.1415926;
	while(apha<-3.14)
		apha += 2*3.1415926;
	//cmd.angular.z = 0.25*apha;
	cmd.angular.z = 0.3*sina*cosa+0.6*apha;
	if (cmd.angular.z > 0.5)
		cmd.angular.z = 0.5;
	if(goalflag>0)
	{
		cmd.linear.x = 0;
		cmd.angular.z = 0;
	}
	cmd_vel_publisher_.publish(cmd);
}


int main(int argc, char **argv) 
{ 
	//Initiate ROS  
	ros::init(argc, argv, "path_pursuit"); 
	//Create an object of class Pursuit that will take care of everything 
	ros::NodeHandle nh;
 	ros::NodeHandle nh_private("~");
  
 	Pursuit node(nh, nh_private); 

 	// ros::Rate loop_rate(10);

 	// while (ros::ok())
 	// {
	// 	//ROS_INFO("444");
	// 	if (pathRead) 		node.calPID(); // publishing cmd_vel without knowing pose is very dangerous
 	// 	ros::spinOnce();
 	// 	loop_rate.sleep();
 	// }
	 ros::spin();
 	
 	return 0; 
} 
