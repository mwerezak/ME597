//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include "pose_util.hpp"

using namespace ros;
using namespace geometry_msgs;

//Callback function for the Position topic 
void pose_update_callback(const PoseWithCovarianceStamped& msg)
{
	//This function is called when a new pose message is received

	double X = POSEST_X(msg.pose); // Robot X position
	double Y = POSEST_Y(msg.pose); // Robot Y position
	double Yaw = tf::getYaw(POSEST_O(msg.pose)); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	init(argc,argv,"main_control");
	NodeHandle node_handle;

	//Subscribe to the desired topics and assign callbacks
	Subscriber pose_sub = node_handle.subscribe("/amcl_pose", 1, pose_update_callback);

	//Setup topics to Publish from this node
	Publisher velocity_publisher = node_handle.advertise<Twist>("/cmd_vel_mux/input/navi", 1);
	
	//Velocity control variable
	Twist vel_cmd;
	
	//Set the loop rate
	Rate loop_rate(20);    //20Hz update rate
    
	while (ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		spinOnce();   //Check for new messages
    
		//velocity_publisher.publish(vel); // Publish the command velocity
		//ROS_DEBUG("Main - Velocity commands: v - %f, w - %f", vel.linear.x, vel.angular.z);
 
	}

	return 0;
}
