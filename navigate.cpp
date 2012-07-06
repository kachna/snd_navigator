#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "snd.h"
#include "snd_data.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>


SND_data data;
std::ofstream file;

//SND_algorithm snd(&data);

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
/*
 * ROS subscriber function. It reads nav_msgs::Odometry msg
 * 
 */
{
 
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  double a = 2*std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  
  data.setPos(x, y, a);
   
  file << x << "," << y << "," << a << std::endl;
 
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
/*
 * ROS subscriber function. It reads sensor_msgs::LaserScan
 * 
 */

{
	std::vector<float> vec;
	std::vector<float>::iterator it;
	vec = msg->ranges;

	for (it = vec.begin(); it < vec.end(); it++) {
		if (*it < msg->range_min)
		   *it = msg->range_min;
		if (*it > msg->range_max)
		   *it = msg->range_max;
	}

	data.setLaserScan(msg->angle_increment, msg->range_max, vec);
	
}

void readParams(ros::NodeHandle * node)
/*
 * It reads SND parametrs and lists of goals from parametr server. 
 * 
 */

{

	double x,y,a;
	node->getParam("robot_radius", data.robot_radius);
	node->getParam("max_speed", data.max_speed);
	node->getParam("max_turn_rate", data.max_turn_rate);
	node->getParam("avoid_dist", data.obstacle_avoid_dist);
	node->getParam("min_gap", data.min_gap_width);

	XmlRpc::XmlRpcValue gX;
	XmlRpc::XmlRpcValue gY;
	XmlRpc::XmlRpcValue gA;

	ROS_INFO("getting goals ...");
	node->getParam("goalsX", gX);
	node->getParam("goalsY", gY);
	node->getParam("goalsA", gA);
	
	//ROS_INFO("parsing goal X - %s ", gX.getType());

	for (int i = 0; i < gX.size(); i++) {
			//std::cout <<" type: " << gX[i].getType() << std::endl;
			std::vector<float> * f = new std::vector<float>();
			f->push_back(static_cast<double>(gX[i]));
			f->push_back(static_cast<double>(gY[i]));
			f->push_back(static_cast<double>(gA[i]));
			data.goal_vector.push_back(*f);
	}
	

	data.setGoal(x, y, a);

	
	ROS_INFO("ROBOT RADUIS: %f", data.robot_radius);
	ROS_INFO("MAX SPEED: %f", data.max_speed);
	ROS_INFO("TURN RATE: %f", data.max_turn_rate);
	ROS_INFO("OBSTACEL AVOIDIN DISTANCE: %f", data.obstacle_avoid_dist);
	ROS_INFO("MIN GAP: %f", data.min_gap_width);
	ROS_INFO("FIRST GOAL: %f, %f, %f PI/rad", data.getGoalX(), data.getGoalY(), data.getGoalA());

	

	
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "snd_navigator");
	ros::NodeHandle node;
	ros::NodeHandle parametr("~");
	
	std::string topic_pose;
	std::string topic_laser;
	std::string topic_speed;


	// read names of topics
	node.param<std::string>("/snd_navigator/topic_pose", topic_pose, "dom");
	node.param<std::string>("/snd_navigator/topic_laser", topic_laser, "base_scan");
	node.param<std::string>("/snd_navigator/topic_speed", topic_speed, "cmd_vel");	
	
	// generate uuid of experiment
	std::string uuid;
	node.param<std::string>("/snd_navigator/uuid", uuid, "-");
	uuid = "pos_data/"+uuid;
	file.open (uuid.c_str());
	
	// read SND parametrs and goals from parametr server
	readParams(&parametr);
	
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>(topic_speed, 1);
	data.setPublisher(&pub);
	
	// read odomoetry messages
	ROS_INFO("Subscribing: %s", topic_pose.c_str());
	ros::Subscriber subs_odom = node.subscribe(topic_pose, 1, positionCallback);
	// read laser scan messages
	ROS_INFO("Subscribing: %s", topic_laser.c_str());
	ros::Subscriber subs_laser = node.subscribe( topic_laser, 1, laserScanCallback);
	

	
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		// run algorithm
		main_algorithm(&data);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	data.publishSpeed(0, 0);
	file.close();
	
	ros::spin();
	
	return 0;
}