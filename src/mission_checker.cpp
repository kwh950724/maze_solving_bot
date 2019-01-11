#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "maze_action_msg/MazeProjectAction.h"
#include "nav_msgs/Odometry.h"
#include "maze_solving_bot/ReqRobotControl.h"
#include <cstring>

class MissionChecker {

protected:
	ros::NodeHandle nh_;

	ros::ServiceClient cli_;
	maze_solving_bot::ReqRobotControl srv_;
	
        ros::Subscriber sub_;
	nav_msgs::Odometry odom_;
	
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
	MissionChecker(std::string name) : as_(nh_, name, boost::bind(&MissionChecker::executeCB, this, _1), false), action_name_(name) {
		cli_ = nh_.serviceClient<maze_solving_bot::ReqRobotControl>("/req_robot_control");
		sub_ = nh_.subscribe("/odom", 1, &MissionChecker::msgCallback, this);
		as_.start();
			
		flag = false;

		ROS_INFO("MISSION CHECKER READY FOR ACTION, WAITING FOR GOAL.");
	}

	~MissionChecker(void) {
	}
	
	void msgCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		cur_xpos_ = msg->pose.pose.position.x;
		cur_ypos_ = msg->pose.pose.position.y;
		odom_ = *msg;

		ROS_INFO("CURRENT XPOS = %f", cur_xpos_);
		ROS_INFO("CURRENT YPOS = %f", cur_ypos_);
	}
	
	void executeCB(const maze_action_msg::MazeProjectGoalConstPtr &goal) {
		ros::Rate loop_rate(10);	
		bool success = true;
		bool is_success = false;
		bool is_preempted = false;	
		
		ROS_INFO("ACTION SERVER	ACTIVATED.");
	
		while (!flag) {
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("ACTION SERVER PREEMPTED.");
				as_.setPreempted();
				is_preempted = true;
				break;	
			}
			
			srv_.request.flag = true;
			
			if (cli_.call(srv_)) {
				ROS_INFO("SERVICE RESPONSE RECEIEVED.");
				if (srv_.response.mission_flag) {
					flag = true;
					is_success = true;
				}	
			}
			else {
				ROS_INFO("SERVICE RESPONSE NOT RECEIEVED.");
			}	
				
			feedback_.odometry = odom_;	
			
			ROS_INFO("SENDING FEEDBACK TO CLIENT.");
			as_.publishFeedback(feedback_);
				
			loop_rate.sleep();
		}
		
		if (is_preempted || is_success) {
			result_.success = success;
			ROS_INFO("GOAL SUCCEEDED, SENDING RESULT");
			as_.setSucceeded(result_); 
		}	
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "mission_checker");
	MissionChecker chkr("/maze_project");
	ros::spin();
}
