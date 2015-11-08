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
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include "frames.h"
#include "occupancy_grid.h"
#include "laser_scan.h"

ros::Publisher map_publisher;

OccupancyGrid occupancy_map(15.0, 15.0, 0.05, tf::Vector3(0.0, 0.0, 0.0));

void scan_callback(const sensor_msgs::LaserScan& msg)
{
	UpdateMapFromScan(occupancy_map, msg);
	
	nav_msgs::OccupancyGrid map_msg;
	occupancy_map.writeToMsg(map_msg);
	map_publisher.publish(map_msg);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"lab2_mapping");
	ros::NodeHandle node;

	//Subscribe to the desired topics and assign callbacks
	ros::Subscriber scan_sub = node.subscribe("/scan", 1, scan_callback);

	//Setup topics to Publish from this node
	map_publisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

	while (ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		loop_rate.sleep(); //Maintain the loop rate
	}

	return 0;
}
