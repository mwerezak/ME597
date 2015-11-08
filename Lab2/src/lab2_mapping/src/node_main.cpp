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
//#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "occupancy_grid.h"
#include "laser_scan.h"

int tick;

//tf::TransformBroadcaster tf_bcaster;
ros::Publisher pose_publisher;
ros::Publisher map_publisher;
//ros::Publisher vision_publisher;

tf::Transform ips_robot;

OccupancyGrid occupancy_map(20.0, 20.0, 0.05, tf::Vector3(0.0, 0.0, 0.0));
//OccupancyGrid vision_map(20.0, 20.0, 0.10, tf::Vector3(0.0, 0.0, 0.0));

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

	int i;
	for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

	tf::poseMsgToTF(msg.pose[i], ips_robot);
	
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "pose";
	pose.pose = msg.pose[i];
	pose_publisher.publish(pose);
	ROS_WARN("pose_callback X: %f Y: %f Yaw: %f", 
				ips_robot.getOrigin().getX(), ips_robot.getOrigin().getY(), 
				tf::getYaw(ips_robot.getRotation()));
	
	//tf_bcaster.sendTransform
	//	(
	//		tf::StampedTransform(ips_robot, ros::Time::now(), "world", "pose")
	//	);
}
#endif


/*
//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
	//This function is called when a new map is received

	//you probably want to save the map into a form which is easy to work with
}
*/

void scan_callback(const sensor_msgs::LaserScan& msg)
{
	//static const tf::Transform null_tf(tf::createQuaternionFromYaw(0.0), tf::Vector3(0, 0, 0));
	
	MappingUpdate(occupancy_map, msg, ips_robot);
	//MappingUpdate(vision_map, msg, null_tf);
	//ROS_WARN("scan_callback");
	
	nav_msgs::OccupancyGrid map_msg;
	occupancy_map.writeToMsg(map_msg);
	map_publisher.publish(map_msg);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle node;

	//Subscribe to the desired topics and assign callbacks
	#ifdef LIVE
	ros::Subscriber pose_sub = node.subscribe("/indoor_pos", 1, pose_callback);
	#else
	ros::Subscriber pose_sub = node.subscribe("/gazebo/model_states", 1, pose_callback);
	#endif
	//ros::Subscriber map_sub = node.subscribe("/map", 1, map_callback);
	ros::Subscriber kinect_sub = node.subscribe("/scan", 1, scan_callback);

	//Setup topics to Publish from this node
	//ros::Publisher velocity_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
	//marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
	
	//tf_bcaster.sendTransform
	//	(
	//		tf::StampedTransform(occupancy_map.getOrigin(), ros::Time::now(), "world", "map")
	//	);

	map_publisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
	
	ros::Publisher marker_publisher = node.advertise<visualization_msgs::Marker>("/robot_marker", 1, true);
	visualization_msgs::Marker ips_points;
	ips_points.header.frame_id = "/map";
	ips_points.id = 0;

	ips_points.type = visualization_msgs::Marker::POINTS;
	ips_points.scale.x = 0.1;
	ips_points.scale.y = 0.1;

	ips_points.color.g = 1.0f;
	ips_points.color.a = 1.0;
	
	ips_points.points.resize(1);
	
	//vision_publisher = node.advertise<nav_msgs::OccupancyGrid>("/vision", 1, true);

	//Initialize the robot position/orientation to some default value
	ips_robot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	ips_robot.setRotation(tf::createQuaternionFromYaw(0.0));

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

	tick = 0;
	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
		
		geometry_msgs::Point p;
		p.x = ips_robot.getOrigin().getX();
		p.y = ips_robot.getOrigin().getY();
		p.z = 0;
		ips_points.points[0] = p;
		
		marker_publisher.publish(ips_points);
		/*
		if(tick % 20 == 0)
		{
			occupancy_map.writeToMsg(map_msg);
			map_publisher.publish(map_msg);
		}
		*/
		
		tick++;
	}

	return 0;
}
