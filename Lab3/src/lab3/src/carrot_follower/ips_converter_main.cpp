
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

const double IPS_SCALE_CORRECTION = 2.17;

ros::Publisher vo_publisher;

tf::Transform last_ips, last_odom_offset;

tf::Transform init_pose;
bool init = false;

#ifdef SIMULATION
void ConvertIPS(const gazebo_msgs::ModelStates& msg)
{
	//search the ModelStates struct for the pose.
	for(int i = 0; i < msg.name.size(); i++) 
	{
		if(msg.name[i] == "mobile_base")
		{
			if(!init)
			{
				tf::poseMsgToTF(msg.pose[i], init_pose);
				init = true;
			}
			
			geometry_msgs::PoseWithCovariance gazebo_pose;
			//gazebo_pose.pose = stamped_pose.pose;
			gazebo_pose.pose = msg.pose[i];
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
			vis_odom.header.frame_id = "world";
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
	tf::Transform ips_tf;
	tf::poseMsgToTF(ips_msg.pose.pose, ips_tf);
	
	if(!init)
	{
		init_pose = ips_tf;
		init = true;
	}
	
	//scale correction
	tf::Vector3 ips_pos = ips_tf.getOrigin();
	ips_pos *= IPS_SCALE_CORRECTION;
	ips_pos.setZ(0.0);
	ips_pos.setX(-ips_pos.getX());

	nav_msgs::Odometry vis_odom;
	vis_odom.header.seq++;
	vis_odom.header.stamp = ros::Time(0);
	vis_odom.header.frame_id = "world";
	tf::pointTFToMsg(ips_pos, vis_odom.pose.pose.position);
	vis_odom.pose.covariance = ips_msg.pose.covariance;
	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			if(i == j && vis_odom.pose.covariance[6*i+j] == 0)
			{
				vis_odom.pose.covariance[6*i+j] = 0.025;
				vis_odom.twist.covariance[6*i+i] = 40000;
			}
			vis_odom.twist.covariance[6*i+i] = 0;
		}
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
		
		if(init)
		{
			tf_bcaster.sendTransform(tf::StampedTransform(init_pose, ros::Time::now() + ros::Duration(10), "world", "odom"));
			tf_bcaster.sendTransform(tf::StampedTransform(ident, ros::Time::now(), "world", "map"));
		}
		
		loop_rate.sleep(); //Maintain the loop rate
	}
}
