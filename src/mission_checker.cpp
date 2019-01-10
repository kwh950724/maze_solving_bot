#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "maze_action_msg/MazeProjectAction.h"
#include "nav_msgs/Odometry.h"
#include "maze_solving_bot/ReqRobotControl.h"
#include <cstring>

class MissionChecker {

private:
	ros::NodeHandle nh_;

	ros::ServiceClient cli_;
	maze_solving_bot::ReqRobotContorl srv_;
	
        ros::Subscriber sub_;
	
	actionlib::SimpleActionServer<maze_action_msg::MazeProjectAction> as_;
	maze_action_msg::MazeProjectFeedback feedback_;
	maze_action_msg::MazeProjectResult result_;
	std::string action_name_;

	float min_dist_;
	
	float goal_xpos_;
	float goal_ypos_;
	
	float cur_xpos_;
	float cur_ypos_;

	bool flag;

public:
	MissionChecker(void) : as_(nh_, name, boost::bind(&MazeProjectAction::excuteCB, this, _1), false), action_name_(name) {
		cli_ = nh_.serviceClient<maze_solving_bot::ReqRobotControl>("/req_robot_control");
		sub_ = nh_.subscribe("/odom", 1, msgCallback);
		as_.start();
		flag = false;
	}

	~MissionChecker(void) {
	}
	
	void msgCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		cur_x_pos_ = msg->pose.pose.position.x;
		cur_y_pos_ = msg->pose.pose.position.y;
		feed_back_.odometry = *msg;

		ROS_INFO("CURRENT XPOS = %f", cur_x_pos_);
		ROS_INFO("CURRENT YPOS = %f", cur_y_pos_);

		isArrived(cur_x_pos_, cur_y_pos_);
	}

	void executeCB(const maze_action_msg::MazeProjectGoalConstPtr &goal) {
		
		ROS_INFO("ACTION SERVER	ACTIVATED.");
	
		while (!flag) {
			
		}		
	}

	bool isArrived(float cur_xpos, float cur_ypos) {
		float dist = sqrt((goal_xpos_ - cur_xpos)*(goal_xpos_ - cur_xpos) + (goal_ypos_ - cur_ypos)*(goal_ypos_ - cur_ypos));
		
		if (distance <= min_dist_) {
			ROS_INFO("SUCCESSFULLY ARRIVED AT DESTINATION.");
			flag = true;
			return true;
		}
		else {
			ROS_INFO("MAZEBOT ON THE WAY TO DESTINATION.");
			return false;
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "mission_checker");
	MissionChecker chkr;
	ros::spin();
}
