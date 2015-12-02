
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

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
			static tf::TransformListener tf_listener;
			tf::StampedTransform odom_tf;
			try
			{
				tf_listener.lookupTransform("world", "odom", ros::Time(0), odom_tf);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				continue;
			}
			
			tf::Stamped<tf::Transform> ips_pose;
			tf::poseMsgToTF(msg.pose[i], ips_pose);
			
			ips_pose *= odom_tf;
			
			geometry_msgs::PoseWithCovariance gazebo_pose;
			tf::poseTFToMsg(ips_pose, gazebo_pose.pose.pose);
			for(int i = 0; i < 6; i++)
			{
				for(int j = 0; j < 6; j++)
				{
					gazebo_pose.pose.covariance[6*i+j] = (i == j? 0.1 : 0.0);
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
	
	tf::Transform ips_tf;
	tf::poseMsgToTF(ips_msg.pose.pose, ips_tf);
	
	if(!init)
	{
		ips_origin_offset = ips_tf;
		ips_tf.setIdentity();
		init = true;
	}
	else
	{
		ips_tf *= ips_origin_offset.inverse();
	}
	
	nav_msgs::Odometry vis_odom;
	vis_odom.header = ips_msg.header;
	
	tf::poseTFToMsg(ips_tf, vis_odom.pose.pose);
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
	
	ros::spin();
}
