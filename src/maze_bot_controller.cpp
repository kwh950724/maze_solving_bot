#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "maze_solving_bot/ReqRobotControl.h"

class Mazebot {
protected:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	ros::ServiceServer serv_;
	geometry_msgs::Twist msg_;
	sensor_msgs::LaserScan scan_;

	float linear_x_;
	float angular_z_;
	
	// float range variable
	

public:
	Mazebot(void) {
		pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		sub_ = nh_.subscribe("/scan", 1, &Mazebot::msgCallback, this);
		serv_ = nh_.advertiseService("/req_robot_control", &Mazebot::manipulation, this);
		ROS_INFO("MAZEBOT READY FOR ACTION.");
	}

	~Mazebot(void) {
	}
	
	void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		// calculate && renew linear_x_, angular_x_;
		scan_ = *msg;
		wallFollow(scan_);
	}

	bool manipulation(maze_solving_bot::ReqRobotControl::Request &req, maze_solving_bot::ReqRobotControl::Response &res) {
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
			ROS_INFO("INVALID REQUEST, SERVICE REQUEST NOT RECEIEVED.");
			return false;
		}
	}
	
	// maze solving algorithm
	void wallFollow(const sensor_msgs::LaserScan scan) {
		float head_dist = scan.ranges[0];
		if (head_dist < 0.6) {
			linear_x_ = 0;
			angular_z_ = 0.2;
			ROS_WARN("WALL AHEAD, EMERGENCY STOP.");
		}
		else {
			linear_x_ = 0.3;
			angular_z_ = 0;
		}
				
		ROS_INFO("VELOCITY, STEER UP TO DATE.");	
	}	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "maze_bot_controller");
	Mazebot bot;
	ros::spin();

	return 0;
}
