#ifndef SND_DATA_H
#define SND_DATA_H


#include <vector>

#include "spaces.h"

#include <ros/ros.h>


typedef unsigned int uint32_t;


class SND_data
{
	private:
		
		double scanRes;
		double maxRange;
		uint32_t count;

		// robot posotion
		double x;
		double y;
		double yaw;
		
		// goal postition
		double goalX,goalY,goalA;
		
		std::vector<float> laser_ranges;
		
		ros::Publisher * publisher;

	public:
		SND_data();
		double robot_radius;
		double min_gap_width;
		double obstacle_avoid_dist;
		double max_speed;
		double max_turn_rate;
		double goal_position_tol;
		double goal_angle_tol;
		std::vector< std::vector<float> > goal_vector;
		
		double   GetScanRes() { return scanRes; };
		double   GetMaxRange() { return maxRange; };
		uint32_t GetCount() { return count; };
		
		double   range(const int index) {return laser_ranges[index]; };

		double GetXPos() { return x; };
		double GetYPos() { return y; };
		double GetYaw()  { return yaw; };
		
		void setPos(double posX, double posY, double posA) {x=posX; y=posY, yaw=posA; };

		double getGoalX() { return goalX; }
		double getGoalY() { return goalY; }
		double getGoalA() { return goalA; }
		
		void setGoal(double x, double y, double a) {goalX = x; goalY = y; goalA = a; };
		void setGoal(std::vector< std::vector<float> > goals) { goal_vector = goals; };
		
		bool hasNextGoal() { if (goal_vector.size() > 0) true; else false; };
		
		void setLaserScan(double res, double range, std::vector<float> scans);
		void setPublisher(ros::Publisher * pub) { publisher = pub; };
		
		void publishSpeed(double driveSpeed, double turnSpeed);
		void WaitForNextGoal();
	/*
		void   SetSpeed(double velocity_modulus,
							double velocity_angle);
		void   SetSpeed(double velocity_x,
							double velocity_y,
							double velocity_angle);
	*/


};

#endif //SND_DATA_H