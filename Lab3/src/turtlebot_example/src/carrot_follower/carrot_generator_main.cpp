
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>



int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"carrot_follower");
	ros::NodeHandle node;
	
	ros::Subscriber carrot_sub = node.subscribe("/carrot", 1, UpdateCarrot);
	ros::Subscriber pose_sub = node.subscribe("/robot_pose", 1, UpdatePose);
	
	nav_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/debug_pose", 1);
	
	ros::Rate loop_rate(20);    //20Hz update rate
	
	while (ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		
		
		loop_rate.sleep(); //Maintain the loop rate
	}
}
