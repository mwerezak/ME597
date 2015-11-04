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
#include <angles/angles.h>

#include "pose_util.hpp"
#include "SimpleLocalPlanner.hpp"

using namespace std;
using namespace ros;
using namespace geometry_msgs;


bool has_init = false; //wait until after we've recieved our first position update before computing commands
SimpleLocalPlanner motion_planner;

void create_square_plan(vector<PoseStamped>& plan, double initx, double inity, double sidelen)
{
	PoseStamped pose;
	
	init2DNavPose(pose, sidelen + initx, 0.0 + inity, 0.0);
	plan.push_back(pose);
	
	init2DNavPose(pose, sidelen + initx, sidelen + inity, 0.0);
	plan.push_back(pose);
	
	init2DNavPose(pose, 0.0 + initx, sidelen + inity, 0.0);
	plan.push_back(pose);
	
	init2DNavPose(pose, 0.0 + initx, 0.0 + inity, 0.0);
	plan.push_back(pose);
}

//Callback function for the Position topic 
void pose_update_callback(const PoseWithCovarianceStamped& msg)
{
	//This function is called when a new pose message is received

	double X = POSEST_X(msg.pose); // Robot X position
	double Y = POSEST_Y(msg.pose); // Robot Y position
	double Yaw = tf::getYaw(POSEST_O(msg.pose)); // Robot Yaw
	
	motion_planner.updateLatestPose(msg);
	if(!has_init)
	{
		vector<PoseStamped> square_plan;
		create_square_plan(square_plan, X, Y, 0.75);
		motion_planner.setPlan(square_plan);
		has_init = true;
	}

	/*
	ROS_WARN
		(
			"pose_callback X: %f Y: %f Yaw: %fdeg TS: %0.4f", 
			X, Y, angles::to_degrees(angles::normalize_angle_positive(Yaw)), 
			msg.header.stamp.toSec()
		);
	*/
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	init(argc,argv,"main_control");
	NodeHandle node_handle;

	//Subscribe to the desired topics and assign callbacks
	Subscriber pose_sub = node_handle.subscribe("/indoor_pos", 1, pose_update_callback);

	//Setup topics to Publish from this node
	Publisher velocity_publisher = node_handle.advertise<Twist>("/cmd_vel_mux/input/navi", 1);
	
	//Initialize the motion planner
	motion_planner.turn_rate = 0.2;
	motion_planner.traj_tolerance = angles::from_degrees(10.0);
	motion_planner.goal_tolerance = 0.15;
	
	//Velocity control variable
	Twist vel_cmd;
	
	//Set the loop rate
	Rate loop_rate(20);    //20Hz update rate
    
	while (ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		spinOnce();   //Check for new messages
		
		if(!has_init || motion_planner.isGoalReached() || !motion_planner.computeVelocityCommands(vel_cmd))
		{
			TWIST_FWD(vel_cmd) = 0.0; //stop
			TWIST_TURN(vel_cmd) = 0.0;
		}
		
		velocity_publisher.publish(vel_cmd); // Publish the command velocity
		ROS_ERROR("Main - Velocity commands: v - %f, w - %f", vel_cmd.linear.x, vel_cmd.angular.z);
		
		
 
	}

	return 0;
}
