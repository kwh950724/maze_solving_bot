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
	float max_range_;
	float min_range_;
public:
	Mazebot(void) {
		pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		sub_ = nh_.subscribe("/scan", 1, &Mazebot::msgCallback, this);
		serv_ = nh_.advertiseService("/req_robot_control", &Mazebot::manipulation, this);
		max_range_ = 2.5;
		min_range_ = 0.5;
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
		int left_count = 0;
		int right_count = 0;
		int left_clash_count = 0;
		int right_clash_count = 0;

		for (int i = 23; i < 68; i += 22) {
			if (scan.ranges[i] < max_range_ && scan.ranges[i] > min_range_) {
				left_count += 1;
			} 	
			if (scan.ranges[i] < min_range_) {
				left_clash_count += 1;
			} 
		}

		for (int i = 293; i < 338; i += 22) {
			if (scan.ranges[i] < max_range_ && scan.ranges[i] > min_range_) {
				right_count += 1;
			}
			if (scan.ranges[i] < min_range_) {
				right_clash_count += 1;
			} 
		}

		ROS_INFO("LEFT COUNT = %d, LEFT CLASH COUNT = %d", left_count, left_clash_count);
		ROS_INFO("RIGHT COUNT = %d, RIGHT CLASH COUNT = %d", right_count, right_clash_count);
	
		if (left_count == right_count) {
			ROS_INFO("GO STRAIGHT.");
			linear_x_ = 0.3;
			angular_z_ = 0;
		}
		else if (left_count < right_count) {
			ROS_INFO("TURN LEFT.");
			linear_x_ = 0.2;
			angular_z_ = -0.3;
		}
		else {
			ROS_INFO("TURN RIGHT.");
			linear_x_ = 0.2;
			angular_z_ = 0.3;
		}

		if (left_clash_count > 0) {
			ROS_WARN("LEFT WALL IS TOO CLOSE.");
			linear_x_ = 0.2;
			angular_z_ = 0.2; 
		}
		
		if (right_clash_count > 0) {
			ROS_WARN("RIGHT WALL IS TOO CLOSE.");
			linear_x_ = 0.2;
			angular_z_ = -0.2;
		}		

		ROS_INFO("VELOCITY, STEER UP TO DATE.");	
	}	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "maze_bot_controller");
	Mazebot bot;
	ros::spin();

	return 0;
};
