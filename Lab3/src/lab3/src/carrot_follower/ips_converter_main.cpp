
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include "../common.h"

ros::Publisher vo_publisher;
tf::Transform ips_origin_offset;
bool init = false;

#ifdef SIMULATION
void ConvertIPS(const gazebo_msgs::ModelStates& msg)
{
	//search the ModelStates struct for the pose.
	for(int i = 0; i < msg.name.size(); i++) 
	{
		if(msg.name[i] == "mobile_base")
		{
			geometry_msgs::PoseStamped stamped_pose;
			stamped_pose.header.seq++;
			stamped_pose.header.stamp = ros::Time(0);
			stamped_pose.header.frame_id = "world";
			stamped_pose.pose = msg.pose[i];
			
			if(!convert_pose(stamped_pose, stamped_pose, "/odom")) continue;
			
			geometry_msgs::PoseWithCovariance gazebo_pose;
			gazebo_pose.pose = stamped_pose.pose;
			for(int i = 0; i < 6; i++)
			{
				for(int j = 0; j < 6; j++)
				{
					gazebo_pose.covariance[6*i+j] = (i == j? 0.1 : 0.0);
				}
			}
			
			nav_msgs::Odometry vis_odom;
			vis_odom.header.seq++;
			vis_odom.header.stamp = ros::Time::now();
			vis_odom.header.frame_id = "odom";
			vis_odom.pose = gazebo_pose;
			for(int i = 0; i < 6; i++)
			{
				vis_odom.twist.covariance[6*i+i] = 40000;
			}

			vo_publisher.publish(vis_odom);
		}
	}
}
#else
void ConvertIPS(const geometry_msgs::PoseWithCovarianceStamped& ips_msg)
{
	//TODO compensate for IPS weirdness
	
	geometry_msgs::PoseStamped stamped_pose;
	stamped_pose.header = ips_msg.header;
	stamped_pose.pose = ips_msg.pose.pose;

	if(!convert_pose(stamped_pose, stamped_pose, "/odom")) return;

	nav_msgs::Odometry vis_odom;
	vis_odom.header = ips_msg.header;
	vis_odom.pose.pose = stamped_pose.pose;
	vis_odom.pose.covariance = ips_msg.pose.covariance;
	for(int i = 0; i < 6; i++)
	{
		vis_odom.twist.covariance[6*i+i] = 40000;
	}
	
	vo_publisher.publish(vis_odom);
}
#endif

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"ips_reader");
	ros::NodeHandle node;
	
	ros::Subscriber ips_sub = node.subscribe("/indoor_pos", 1, ConvertIPS);
	vo_publisher = node.advertise<nav_msgs::Odometry>("/vo", 1);
	
	//broadcast map->odom tf
	tf::TransformBroadcaster tf_bcaster;
	tf::Transform ident;
	ident.setIdentity();
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		
		tf_bcaster.sendTransform(tf::StampedTransform(ident, ros::Time::now(), "world", "odom"));
		tf_bcaster.sendTransform(tf::StampedTransform(ident, ros::Time::now(), "world", "map"));
		
		loop_rate.sleep(); //Maintain the loop rate
	}
}
