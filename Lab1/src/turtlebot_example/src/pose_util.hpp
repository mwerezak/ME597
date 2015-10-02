#ifndef POSE_UTIL_HPP
#define POSE_UTIL_HPP

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace geometry_msgs;

// Convenience macros
#define POSEST_X(pose_stamped) (pose_stamped.pose.position.x)
#define POSEST_Y(pose_stamped) (pose_stamped.pose.position.y)
#define POSEST_Z(pose_stamped) (pose_stamped.pose.position.z)
#define POSEST_O(pose_stamped) (pose_stamped.pose.orientation)

static void init2DNavPose(PoseStamped& pose, double x, double y, double theta)
{
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	
	POSEST_X(pose) = x;
	POSEST_Y(pose) = y;
	POSEST_Z(pose) = 0.0;
	POSEST_O(pose) = tf::createQuaternionMsgFromYaw(theta);
}

#endif