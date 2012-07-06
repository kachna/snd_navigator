/*
 *      snd_data.cpp
 *
 *      Based on SND driver by Joey Durham <joey@engineering.ucsb.edu> and Luca Invernizzi <invernizzi.l@gmail.com>
 *      for Plaer/Stage project.
 * 
 *      Coded for ROS by:   Petr Martinec <petr.martinec@gmail.com>
 *
 *      program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>

#include <vector>

#include <ros/console.h>
#include <ros/ros.h>

#include "snd_data.h"


SND_data::SND_data() {
	scanRes = 0;
	maxRange = 0;
	count = 0;

	x = 0;
	y = 0;
	yaw = 0;
	
	robot_radius = 0.1;
	obstacle_avoid_dist = 0.03;
	max_speed = 1.0;
	max_turn_rate = 0.8;
	min_gap_width = 2.2*robot_radius;	
	goal_position_tol = 0.5*robot_radius;
	goal_angle_tol = 15;
	
	
}


void SND_data::publishSpeed(double driveSpeed, double turnSpeed) {
	
	geometry_msgs::Twist msg;
	
	msg.linear.x = driveSpeed;
	msg.linear.y = 0;
	msg.linear.z = 0;
	
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = turnSpeed;

	publisher->publish(msg);
	//ROS_INFO("publishing speed: %f, %f", driveSpeed, turnSpeed);
		
}

void SND_data::setLaserScan(double res, double range, std::vector<float> scans) {
	scanRes = res;
	maxRange = range;
	laser_ranges = scans;
	count = scans.size();
	
}

void SND_data::WaitForNextGoal() {
	
	ROS_INFO("REACHED GOAL %f %f %f", goal_vector[0][0], goal_vector[0][1], goal_vector[0][2]);
	goal_vector.erase(goal_vector.begin());
	
	if (!this->hasNextGoal()) {
		ROS_INFO("NO MORE GOALS, %d", goal_vector.size());
		this->publishSpeed(0,0);
		ros::shutdown();
	}
}

void SND_data::exit() {
	
	ROS_INFO("Exiting ...");
	ros::shutdown();
}