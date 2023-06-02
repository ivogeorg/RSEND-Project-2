#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// (ivogeorg) NOTES:
// This is a node "drive_bot" which provides a service 
// "/ball_chaser/command_robot"to drive the robot through a (global) 
// "motor_command_publisher". It takes two velocity parameters 
// (linear x, angular z) in the "ball_chaser::DriveToTarget" request, 
// publishes them with "moto_command_publisher", and returns a 
// feedback message in the "ball_chaser::DriveToTarget" response.
// The message is also output on the terminal.

// ROS publisher of motor commands
ros::Publisher motor_command_publisher;

// (ivogeorg) DONE: 
//       Create handle_drive_request callback that executes whenever a drive_bot
//       service is requested. It should publish the requested linear_x and
//       angular_z velocities to the robot wheel joints, and return a feedback
//       message with the requested wheel velocities.
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {

	// Create a motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;

	// Set wheel velocities from request
	motor_command.linear.x = req.linear_x;
	motor_command.angular.z = req.angular_z;

	// Publish angles in a command to drive the robot
	motor_command_publisher.publish(motor_command);

	// Wait 3 seconds for joints to settle (TODO: Is this necessary?)
	//ros::Duration(3).sleep();
	
	res.msg_feedback = "Robot drive command (lin_x, ang_z): " + std::to_string(motor_command.linear.x) + \
					   ", " + std::to_string(motor_command.angular.z);
	ROS_INFO_STREAM(res.msg_feedback);


	return true;
}

int main(int argc, char** argv) {

	// Initialize a ROS node
	ros::init(argc, argv, "drive_bot");

	// Create a ROS NodeHandle object
	ros::NodeHandle n;

	// Inform ROS master that messages of type geometry_msgs::Twist will be
	// published on the robot actuation topic with a publishing queue size
	// of 10
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// (ivogeorg) DONE:
	//		 Define a drive /ball_chaser/command_robot service with a 
	//       handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to command robot");

	// (ivogeorg) DONE:
	//       Delete the loop, move the code to the inside of the callback
	//       function and make the necessary changes to publish the requested
	//       velocities instead of constant values

	// Handle ROS communication events
	ros::spin();

	return 0;
}
