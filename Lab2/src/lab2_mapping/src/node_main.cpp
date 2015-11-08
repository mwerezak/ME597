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
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "frames.h"
#include "occupancy_grid.h"
#include "laser_scan.h"

int tick;

ros::Publisher pose_publisher;
ros::Publisher map_publisher;

tf::Transform ips_robot;

OccupancyGrid occupancy_map(15.0, 15.0, 0.05, tf::Vector3(0.0, 0.0, 0.0));

#ifdef LIVE
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	tf::poseMsgToTF(msg.pose.pose, ips_robot);
	
	
	//ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", 
	//			ips_robot.getOrigin().getX(), ips_robot.getOrigin().getY(), 
	//			tf::getYaw(ips_robot.getRotation()));
}
#else
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
	//search the ModelStates struct for the pose.
	geometry_msgs::Pose robot_pose;
	for(int i = 0; i < msg.name.size(); i++) 
		if(msg.name[i] == "mobile_base") 
			robot_pose = msg.pose[i];

	//Update ips_robot
	tf::poseMsgToTF(robot_pose, ips_robot);

	//Publish the pose for Rviz.
	geometry_msgs::PoseStamped pose;
	pose.header.seq++; //somehow this works. Frigging magic.
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = WORLD_FRAME;
	pose.pose = robot_pose;
	pose_publisher.publish(pose);
	
	//ROS_WARN("pose_callback X: %f Y: %f Yaw: %f", 
	//			ips_robot.getOrigin().getX(), ips_robot.getOrigin().getY(), 
	//			tf::getYaw(ips_robot.getRotation()));
}
#endif

void scan_callback(const sensor_msgs::LaserScan& msg)
{
	//static const tf::Transform null_tf(tf::createQuaternionFromYaw(0.0), tf::Vector3(0, 0, 0));
	
	UpdateMapFromScan(occupancy_map, msg);
	//MappingUpdate(vision_map, msg, null_tf);
	//ROS_WARN("scan_callback");
	
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
	#ifdef LIVE
	ros::Subscriber pose_sub = node.subscribe("/indoor_pos", 1, pose_callback);
	#else
	ros::Subscriber pose_sub = node.subscribe("/gazebo/model_states", 1, pose_callback);
	#endif

	ros::Subscriber scan_sub = node.subscribe("/scan", 1, scan_callback);

	//Setup topics to Publish from this node
	pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

	map_publisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

	//Initialize the robot position/orientation to some default value
	ips_robot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	ips_robot.setRotation(tf::createQuaternionFromYaw(0.0));

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

	while (ros::ok())
	{
		//This is dumb, but apparently ROS requires you to constantly
		//resend frames, even if they never change?
		static tf::TransformBroadcaster tf_bcaster;
		tf_bcaster.sendTransform
			(
				tf::StampedTransform
					(
						occupancy_map.getOrigin(), 
						ros::Time::now(), 
						WORLD_FRAME, MAP_FRAME
					)
			);
		
		ros::spinOnce();   //Check for new messages
		loop_rate.sleep(); //Maintain the loop rate
	}

	return 0;
}
