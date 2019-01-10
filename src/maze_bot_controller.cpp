#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "maze_solving_bot/ReqRobotControl.h"

class Maze_bot {
private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	ros::ServiceServer serv_;
	geometry_msgs::Twist msg_;

	float linear_x_;
	float angular_z_;
	
	// float range variable
	

public:
	Maze_bot(void) {
		pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		sub_ = nh_.subscribe("/scan", 1, msgCallback);
		serv_ = nh_.serviceServer("/req_robot_control", manipulation);
	
		ROS_INFO("MAZEBOT READY FOR ACTION.");
	}
	
	void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		// calculate && renew linear_x_, angular_x_;
		wallFollow();
	}

	bool manipulation(maze_solving_bot::ReqRobotControl::Request &req, maze_solving_bot::ReqRobotControl::Request &res) {
		if (req.flag == true) {
			// publish /cmd_vel to bot.
			msg_.linear.x = linear_x_;
			msg_.angular.z = angular_z_;
	
			res.linear_x = linear_x_;
			res.angular_z = angular_z_;
		
			ROS_INFO("VELOCITY: %f", linear_x_);
			ROS_INFO("STEER: %f", angular_z_); 
		
			pub_.publish(msg_);
		
			ROS_INFO("SUCCESSFULLY MANIPULATE MAZEBOT.");
			return true;
		}
		else {
			ROS_INFO("INVALID REQUEST.");
			return false;
		}
	}
	
	// maze solving algorithm
	void wallFollow() {
		
	}	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "maze_controller");
	Maze_bot bot;
	ros::spin()

	return 0;
}