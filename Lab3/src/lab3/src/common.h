#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

bool convert_pose
	(
		const geometry_msgs::PoseStamped& pose_msg,
		geometry_msgs::PoseStamped& converted_pose_msg,
		std::string target_frame
	);


#endif