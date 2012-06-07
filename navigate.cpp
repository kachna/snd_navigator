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


SND_data data;
//SND_algorithm snd(&data);

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  double a = 2*std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  
  data.setPos(x, y, a);
  
  ROS_INFO("Position: [%f, %f, %f]", x, y, a);
  
  //main_algorithm(&data);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   ROS_INFO("INCOMMING laser data.");
	std::vector<float> vec;
	std::vector<float>::iterator it;
	vec = msg->ranges;

	for (it = vec.begin(); it < vec.end(); it++) {
		if (*it < msg->range_min)
		   *it = msg->range_min;
		if (*it > msg->range_max)
		   *it = msg->range_max;
	}
/*	
	   
		if (*it < msg->range_min || *it > msg->range_max || *it < 0.0001) {
			vec.erase(it);
			printf("removed(max: %f) %f", msg->range_max, *it);
		}
*/
	printf("laser - max: %f, incre: %f", msg->range_max, msg->angle_increment);
	data.setLaserScan(msg->angle_increment, msg->range_max, vec);
	
	ROS_INFO("Scanned.");
}

void readParams(ros::NodeHandle * node) {

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
			std::cout <<" type: " << gX[i].getType() << std::endl;
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
	ROS_INFO("FIRST GOAL: %f, %f, %f PI/rad", data.getGoalX(), data.getGoalY(), data.getGoalA());

	

	
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "snd_navigator");
	ros::NodeHandle node;
	ros::NodeHandle parametr("~");
	
	std::string topic_pose;
	std::string topic_laser;
	std::string topic_speed;

	node.param<std::string>("topic_pose", topic_pose, "odom");
	//node.getParam("topic_pose", topic_pose);
	node.param<std::string>("topic_laser", topic_laser, "base_scan");
	node.param<std::string>("topic_speed", topic_speed, "cmd_vel");	
	
	readParams(&parametr);
	
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>(topic_speed, 1);
	data.setPublisher(&pub);
	
	// read odomoetry messages
	//ros::Subscriber subs_odom = node.subscribe("base_pose_ground_truth", 100, positionCallback);
	ROS_INFO("Subscribing: %s", topic_pose.c_str());
	ros::Subscriber subs_odom = node.subscribe(topic_pose, 1, positionCallback);
	// read laser scan messages
	ROS_INFO("Subscribing: %s", topic_laser.c_str());
	ros::Subscriber subs_laser = node.subscribe( topic_laser, 1, laserScanCallback);
	

	
	//ROS_INFO("SND ready...");
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		main_algorithm(&data);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	data.publishSpeed(0, 0);
	
	
	ros::spin();
	
	return 0;
}