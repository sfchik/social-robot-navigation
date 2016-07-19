#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>


#ifndef HUMAN_FOLLOW_GOAL_H
#define HUMAN_FOLLOW_GOAL_H

class HumanFollowGoal
{
	public:
		HumanFollowGoal(const ros::NodeHandle &node);	
		~HumanFollowGoal();
		bool init_;

		void runHumanFollow();
		void callbackPeoplePose(const pedsim_msgs::TrackedPersons::ConstPtr& msg);
		void callbackRobotPose(const nav_msgs::Odometry::ConstPtr& msg);

	private:
		bool tracked_;
		bool ready_;

		ros::NodeHandle nh_;
		ros::Subscriber sub_people_pos_;
		ros::Subscriber sub_robot_pos_;

  	move_base_msgs::MoveBaseGoal goal;
		float pos_robot_x, pos_robot_y;
		float goal_x, goal_y, goal_heading, goal_radius;  //load value from param server
		float leader_angle_score, leader_distance_score, leader_speed_score;
		float leader_angle_max, leader_distance_max, leader_speed_max, leader_speed_min;
		int tracked_id;
};


#endif
