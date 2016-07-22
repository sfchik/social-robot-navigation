#include <ros/ros.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <simple_navigation_goals/human_follow_goal.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

HumanFollowGoal::HumanFollowGoal(const ros::NodeHandle &node)
  : nh_(node)
{
	init_ = false;

	//setup subscriber
	sub_people_pos_ = nh_.subscribe("/pedsim/tracked_persons", 1,  &HumanFollowGoal::callbackPeoplePose, this);
	sub_robot_pos_ = nh_.subscribe("/pedsim/robot_position", 1, &HumanFollowGoal::callbackRobotPose, this);

	nh_.getParam("/human_follow_goal/goal_x", goal_x);
	nh_.getParam("/human_follow_goal/goal_y", goal_y);
	nh_.getParam("/human_follow_goal/goal_heading", goal_heading);
	nh_.getParam("/human_follow_goal/goal_radius", goal_radius);
	nh_.getParam("/human_follow_goal/leader_angle_score", leader_angle_score);
	nh_.getParam("/human_follow_goal/leader_distance_score", leader_distance_score);
	nh_.getParam("/human_follow_goal/leader_speed_score", leader_speed_score);
	nh_.getParam("/human_follow_goal/leader_angle_max", leader_angle_max);
	nh_.getParam("/human_follow_goal/leader_distance_max", leader_distance_max);
	nh_.getParam("/human_follow_goal/leader_speed_max", leader_speed_max);
	nh_.getParam("/human_follow_goal/leader_speed_min", leader_speed_min);

	//initializing parameters
	tracked_ = false;
	ready_ = false;
	goal_ori_ = false;
	goal_same_ = false;

	init_ = true;
}

HumanFollowGoal::~HumanFollowGoal()
{

}

void HumanFollowGoal::runHumanFollow()
{
	ros::Rate r(2);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
		
		if(ready_ == true) {
			if(goal_same_ == false) 
				ac.sendGoal(goal);
		}
	}
}

void HumanFollowGoal::callbackPeoplePose(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	geometry_msgs::Quaternion quat;
	geometry_msgs::PoseStamped msg_convert;
	double yaw;
	float diff_x, diff_y; 
	float score_distance, score_angle;
	vector<int> candidate;
	float score = 0.0;

	if(tracked_ == true) {
		yaw = tf::getYaw(msg->tracks[leader_id].pose.pose.orientation); //ped heading towards robot goal
		diff_x = msg->tracks[leader_id].pose.pose.position.x - pos_robot_x;
		if(yaw > 1.047 || yaw < -1.047) {
			tracked_ = false;
		}
	}	
		
	if(tracked_ == false) {
		for(char i = 0; i < msg->tracks.size(); i++) {
			tracked_ = false;

			yaw = tf::getYaw(msg->tracks[i].pose.pose.orientation); //ped heading towards robot goal	
			diff_x = msg->tracks[i].pose.pose.position.x - pos_robot_x;
		
			if(diff_x > 0.0 && diff_x < leader_distance_max) { //ped is infront of robot
				if(yaw < 1.047 && yaw > -1.047) {  //10 deg
					candidate.push_back(i);
				}
			}
		}

		for(unsigned int i = 0; i < candidate.size(); i++) {  //i has to be unsigned int since vec.size return unsigned int
			diff_x = msg->tracks[candidate[i]].pose.pose.position.x - pos_robot_x;
			diff_y = msg->tracks[candidate[i]].pose.pose.position.y - pos_robot_y;
			float len = sqrt(diff_x*diff_x + diff_y*diff_y);
			float angle = atan(diff_y/diff_x);  //this is the deg bet. line projections to goal and ped
			if(len < leader_distance_max) {
				if(angle < leader_angle_max && angle > -leader_angle_max) {
					score_distance = leader_distance_score*(leader_distance_max - len)/leader_distance_max;	
					score_angle = 	leader_angle_score*(leader_angle_max - abs(angle))/leader_angle_max;
					if(score_distance+score_angle > score)
						leader_id = candidate[i];
					tracked_ = true;
				}
			}
		}
	}		
/*		cout << candidate[i]+1 << " ";
		if(i+1 == candidate.size())
			cout << endl;   */

/*		diff_y = msg->tracks[i].pose.pose.position.y - pos_robot_y;
		len = sqrt(diff_x*diff_x + diff_y*diff_y);
		float deg = atan(diff_y/diff_x);  //this is the deg bet. line projections to goal and ped

//			ROS_INFO("x=%.2f y=%.2f len=%.2f deg=%.2f", diff_x, diff_y, len, deg);
		if(diff_x > 0.0) { 
			if(len < 3.0) {
				if(deg < 0.785 && deg > -0.785) {  //60 deg
					if(yaw < 1.047 && yaw > -1.047) {  //10 deg
						goal.target_pose.pose = msg->tracks[i].pose.pose;	
						tracked_id = i;
						tracked_ = true;	
					}
				}
			}
		}
	} */
	cout << leader_id+1 << endl;
	
	goal.target_pose.header.frame_id = "odom";  //here odom is the global frame
	goal.target_pose.header.stamp = ros::Time::now();

	if(tracked_ == true) {
			quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, goal_heading);
			goal.target_pose.pose.position = msg->tracks[leader_id].pose.pose.position;
			goal.target_pose.pose.orientation = quat;
			ready_ = true;
			goal_ori_ = false;
			goal_same_ = false;	
	} else {
			if(goal_ori_ == false) {
				quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, goal_heading);
				goal.target_pose.pose.position.x = goal_x;
				goal.target_pose.pose.position.y = goal_y;
				goal.target_pose.pose.orientation = quat;
				goal_ori_ = true;
			} else {
				goal_same_ = true;
			}
	}
}


void HumanFollowGoal::callbackRobotPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	pos_robot_x = msg->pose.pose.position.x;
	pos_robot_y = msg->pose.pose.position.y;
}

int main(int argc, char** argv)
{
	//initialize resources
	ros::init(argc, argv, "human_follow_goal");
	ros::NodeHandle node;
	HumanFollowGoal hfg(node);

  if (hfg.init_) {
      ROS_INFO("node initialized, now running ");
      hfg.runHumanFollow();
  }
  else {
      ROS_WARN("Could not initialize simulation, aborting");
      return EXIT_FAILURE;
  }	
}
