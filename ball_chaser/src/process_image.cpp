#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Global service client
ros::ServiceClient client;

// Function calls the command_robot service to drive the robot in the specified direction
// Called by the callback of the images subscription
void drive_robot(float lin_x, float ang_z) {

	// (ivogeorg) DONE: Request a service and pass velocities to it to drive the robot

	// Set the requested velocities
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	// Call the command_robot service
	if (! client.call(srv))
		ROS_ERROR("Failed to call service command_robot");
}

// Callback continuously executes to read image data
void process_image_callback(const sensor_msgs::Image img) {

	int white_pixel = 255;

	// (ivogeorg) DONE:
	// Loop through each pixel in the image and check if there's a bright white one.
	// Then, identify if this pixel falls in the left, mid, or right side of the
	// image. Depending on the position of the "white ball", represented by the
	// white pixel, call the drive_bot function and pass velocities to it. Request
	// a stop when there's no white ball seen by the camera.

	int ball_pixel_index = -1;
	// (ivogeorg) NOTE:
	// sensor_msgs::Image is a flat array where every  "pixel" is a triplet of
	// array indices [r, g, b]. 
	// (0, 0) is at top-left of image and the pixels are at indices [0, h*w-1], 
	// row after row.
	// step = width + row padding, probably for uneven rows 

	// (ivogeorg) Assumptions:
	// 1. step ~ width 
	// 2. r index of the pixel for localization [left, center, right]
	// 3. just one white pixel is enough for correct localization
	for (int i = 0; i < img.height * img.step; i = i + 3) {		// ass 1
		if (img.data[i] == white_pixel && 
			img.data[i + 1] == white_pixel &&
			img.data[i + 2] == white_pixel) {
			ball_pixel_index = i;							// ass 2
			break;
		}
	}

	float lin_x = 0.0, ang_z = 0.0;  						// no ball, stop is default!

	if (ball_pixel_index >= 0) {							// ass 3

		float hor = (float) (ball_pixel_index % img.step); 	// ass 1, horizontal position

		if (hor < 0.33 * img.step) {
			// left
			lin_x = 0.5;									// lin>0, ang>0 is smoother
			ang_z = 0.5;
		} else if (hor < 0.66 * img.step) {
			// mid
			lin_x = 0.5;
		} else {
			// right
			lin_x = 0.5;
			ang_z = -0.5;
		}
	}
	
	drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv) {

	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a command_robot service client
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw to read image data inside process_image_callback
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS comms
	ros::spin();

	return 0;
}
