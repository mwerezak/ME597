#include "common.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

bool convert_pose
(
	const geometry_msgs::PoseStamped& pose_msg, 
	geometry_msgs::PoseStamped& converted_pose_msg, 
	std::string target_frame
)
{
	static tf::TransformListener tf_listener;
	
	tf::StampedTransform odom_tf;
	try
	{
    	tf_listener.waitForTransform(pose_msg.header.frame_id, target_frame, pose_msg.header.stamp, ros::Duration(3.0));
		tf_listener.lookupTransform(pose_msg.header.frame_id, target_frame, pose_msg.header.stamp, odom_tf);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return false;
	}

	tf::Stamped<tf::Transform> pose;
	tf::poseStampedMsgToTF(pose_msg, pose);

	pose *= odom_tf;

	tf::poseStampedTFToMsg(pose, converted_pose_msg);
	return true;
}