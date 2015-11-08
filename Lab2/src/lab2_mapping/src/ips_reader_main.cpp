#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "frames.h"

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
	getTfBroadcaster().sendTransform
		(
			tf::StampedTransform
				(
					ips_tf, ros::Time::now(), WORLD_FRAME, ROBOT_FRAME
				)
		);
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

	ros::spin(); //spin forever - tf updates are processed in the subscriber callbacks.

	return 0;
}