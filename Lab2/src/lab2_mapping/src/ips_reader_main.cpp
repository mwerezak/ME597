#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "frames.h"

const double IPS_SCALE_CORRECTION = 2.17;

ros::Publisher pose_publisher;

tf::TransformBroadcaster& getTfBroadcaster()
{
	static tf::TransformBroadcaster tf_bcaster;
	return tf_bcaster;
}

#ifdef LIVE
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//Callback function for the Position topic (LIVE)
void ips_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	tf::Transform ips_tf;
	tf::poseMsgToTF(msg.pose.pose, ips_tf);
	
	//scale correction
	tf::Vector3 ips_pos = ips_tf.getOrigin();
	ips_pos /= IPS_SCALE_CORRECTION;
	ips_tf.setOrigin(ips_pos);
	
	getTfBroadcaster().sendTransform
		(
			tf::StampedTransform
				(
					ips_tf, ros::Time::now() + TIME_SHIFT, WORLD_FRAME, ROBOT_FRAME
				)
		);
	ROS_WARN("Publishing IPS TF");
	
	geometry_msgs::PoseStamped rviz_pose;
	rviz_pose.header.seq++; //somehow this works. Frigging magic.
	rviz_pose.header.stamp = ros::Time::now() + TIME_SHIFT;
	rviz_pose.header.frame_id = WORLD_FRAME;
	rviz_pose.pose = msg.pose.pose;
	pose_publisher.publish(rviz_pose);
}
#else

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

//Callback function for the Position topic (SIMULATION)
void ips_callback(const gazebo_msgs::ModelStates& msg)
{
	//search the ModelStates struct for the pose.
	geometry_msgs::Pose pose;
	for(int i = 0; i < msg.name.size(); i++) 
		if(msg.name[i] == "mobile_base") 
			pose = msg.pose[i];
	
	tf::Transform ips_tf;
	tf::poseMsgToTF(pose, ips_tf);
	getTfBroadcaster().sendTransform
		(
			tf::StampedTransform
				(
					ips_tf, ros::Time::now(), WORLD_FRAME, ROBOT_FRAME
				)
		);

	//Publish the pose for Rviz.
	geometry_msgs::PoseStamped rviz_pose;
	rviz_pose.header.seq++;
	rviz_pose.header.stamp = ros::Time::now();
	rviz_pose.header.frame_id = WORLD_FRAME;
	rviz_pose.pose = pose;
	pose_publisher.publish(rviz_pose);
}
#endif

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc, argv, "ips_reader");
	ros::NodeHandle node;

	//Subscribe to the desired topics and assign callbacks
	#ifdef LIVE
	ros::Subscriber pose_sub = node.subscribe("/indoor_pos", 1, ips_callback);
	#else
	ros::Subscriber pose_sub = node.subscribe("/gazebo/model_states", 1, ips_callback);
	#endif
	
	//Publish pose for Rviz
	pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

	ros::spin(); //spin forever - tf updates are processed in the subscriber callbacks.

	return 0;
}