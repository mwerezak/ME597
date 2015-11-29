
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher vo_publisher;

#ifdef SIMULATION
void ConvertIPS(const gazebo_msgs::ModelStates& msg)
{
	//search the ModelStates struct for the pose.
	for(int i = 0; i < msg.name.size(); i++) 
	{
		if(msg.name[i] == "mobile_base")
		{
			geometry_msgs::PoseWithCovariance gazebo_pose;
			gazebo_pose.pose.pose = msg.pose[i];
			
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
	nav_msgs::Odometry vis_odom;
	vis_odom.header = ips_msg.header;
	vis_odom.pose = ips_msg.pose;
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
