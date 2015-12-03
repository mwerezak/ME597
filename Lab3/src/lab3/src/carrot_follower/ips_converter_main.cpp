
#include <ros/ros.h>
#include <ros/topic.h>
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

geometry_msgs::PoseWithCovarianceStamped last_odom_msg;
tf::Transform last_ips, last_odom_base;

tf::Transform init_pose;
bool init = false;

void UpdateIPS(const geometry_msgs::PoseWithCovarianceStamped& ips_msg)
{
	tf::poseMsgToTF(ips_msg.pose.pose, last_ips);
	init = true;
	
	tf::Vector3 ips_pos = last_ips.getOrigin();
	ips_pos *= IPS_SCALE_CORRECTION;
	ips_pos.setZ(0.0);
	//ips_pos.setX(-ips_pos.getX());
	last_ips.setOrigin(ips_pos);
	
	tf::Transform last_odom;
	tf::poseMsgToTF(last_odom_msg.pose.pose, last_odom_base); //crap
}

void UpdateOdom(const nav_msgs::Odometry& odom_msg)
{
	if(!init) return;
	
	last_odom_msg.header = odom_msg.header;
	last_odom_msg.pose = odom_msg.pose;
	
	tf::Transform cur_pose;
	//tf::Vector3 cur_loc;
	//tf::Vector3 cur_rot;
	tf::poseMsgToTF(last_odom_msg.pose.pose, cur_pose);
	
	cur_pose *= last_odom_base.inverse();
	cur_pose *= last_ips;
	
	geometry_msgs::PoseStamped cur_pose_msg;
	cur_pose_msg.header.seq++;
	cur_pose_msg.header.stamp = odom_msg.header.stamp;
	cur_pose_msg.header.frame_id = "map";
	tf::poseTFToMsg(cur_pose, cur_pose_msg.pose);
	vo_publisher.publish(cur_pose_msg);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"ips_reader");
	ros::NodeHandle node;
	
	ros::Subscriber ips_sub = node.subscribe("/indoor_pos", 1, UpdateIPS);
	ros::Subscriber odom_sub = node.subscribe("/odom", 1, UpdateOdom);
	vo_publisher = node.advertise<geometry_msgs::PoseStamped>("/turtlebot_pose", 1);
	
	//broadcast map->odom tf
	tf::TransformBroadcaster tf_bcaster;
	tf::Transform ident;
	ident.setIdentity();
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		
		loop_rate.sleep(); //Maintain the loop rate
	}
}
