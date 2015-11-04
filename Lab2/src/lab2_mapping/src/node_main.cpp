//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>


ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{

	int i;
	for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

	ips_x = msg.pose[i].position.x ;
	ips_y = msg.pose[i].position.y ;
	ips_yaw = tf::getYaw(msg.pose[i].orientation);

}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
	//This function is called when a new map is received

	//you probably want to save the map into a form which is easy to work with
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;

	//Subscribe to the desired topics and assign callbacks
	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
	ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

	//Setup topics to Publish from this node
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

	//Velocity control variable
	geometry_msgs::Twist vel;

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate


	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages

		//Main loop code goes here:
		vel.linear.x = 0.1; // set linear speed
		vel.angular.z = 0.3; // set angular speed

		velocity_publisher.publish(vel); // Publish the command velocity
	}

	return 0;
}
