
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include "../common.h"

#define SQR(x) pow(x, 2)

const double GOAL_TOL = 0.1;
const double CARROT_LEAD = 1.0;

ros::Publisher carrot_pub;

nav_msgs::Path current_path;
geometry_msgs::PoseStamped latest_pose;
int goal_idx;
bool init = false;

bool check_goal_reached(const geometry_msgs::Pose& current_loc)
{
	if(goal_idx < 0 || goal_idx >= current_path.poses.size()) return false;
	
	geometry_msgs::Point &current_goal = current_path.poses[goal_idx].pose.position;
	
	//check 2D distance
	double dist_sqr = SQR(current_loc.position.x - current_goal.x) + SQR(current_loc.position.y - current_goal.y);
	//ROS_INFO("Distance to goal sqr: %f", dist_sqr);
	
	return (dist_sqr < SQR(GOAL_TOL));
}

double bearing_to(const geometry_msgs::Point& o, const geometry_msgs::Point& p)
{
	return atan2(p.y - o.y, p.x - o.x);
}

geometry_msgs::Pose generate_carrot_initial(const geometry_msgs::Pose& path_start)
{
	geometry_msgs::Pose carrot;
	carrot.position = path_start.position;
	carrot.orientation = tf::createQuaternionMsgFromYaw(bearing_to(latest_pose.pose.position, path_start.position));
	return carrot;
}

geometry_msgs::Pose generate_carrot_pose(const geometry_msgs::Pose& seg_start, const geometry_msgs::Pose& seg_end)
{
	tf::Transform seg_base;
	tf::Vector3 robot_loc, goal_loc;
	
	tf::poseMsgToTF(seg_start, seg_base);
	seg_base.setRotation(tf::createQuaternionFromYaw(bearing_to(seg_start.position, seg_end.position)));
	
	tf::pointMsgToTF(seg_end.position, goal_loc);
	double seg_len = goal_loc.distance(seg_base.getOrigin());
	
	tf::pointMsgToTF(latest_pose.pose.position, robot_loc);
	robot_loc = seg_base.inverse()(robot_loc);
	//ROS_INFO("Robot position in path segment frame:\n[x: %f, y: %f]", robot_loc.getX(), robot_loc.getY());
	
	double carrot_x = (robot_loc.getX() < seg_len? robot_loc.getX() : seg_len) + CARROT_LEAD;
	tf::Vector3 carrot_loc(carrot_x, 0.0, 0.0);
	
	geometry_msgs::Pose carrot_msg;
	tf::pointTFToMsg(seg_base(carrot_loc), carrot_msg.position);
	carrot_msg.orientation = tf::createQuaternionMsgFromYaw(bearing_to(latest_pose.pose.position, carrot_msg.position));
	return carrot_msg;
}

geometry_msgs::PoseStamped get_next_carrot()
{
	geometry_msgs::PoseStamped new_carrot;
	new_carrot.header.seq++;
	new_carrot.header.stamp = ros::Time::now();
	new_carrot.header.frame_id = "map";
	
	if(goal_idx == 0 || goal_idx == current_path.poses.size() - 1)
	{
		new_carrot.pose = generate_carrot_initial(current_path.poses[goal_idx].pose);
	}
	else
	{
		//TODO: fix driving past final goal
		new_carrot.pose = generate_carrot_pose
			(
				current_path.poses[goal_idx - 1].pose, 
				current_path.poses[goal_idx].pose
			);
	}
	
	return new_carrot;
}

void update_pose(const geometry_msgs::PoseStamped& msg)
{
	latest_pose = msg;
	
	//ROS_INFO("Init is: %d", init);
	if(!init) return;
	
	if(check_goal_reached(msg.pose))
	{
		goal_idx++;
		ROS_INFO("Reached our goal, going to the next one. New goal is: %d of %lu", goal_idx, current_path.poses.size());
	}
	
	if(goal_idx < current_path.poses.size())
	{
		carrot_pub.publish(get_next_carrot());
	}
}

void add_debug_goal(const geometry_msgs::PoseStamped& pose_msg)
{
	geometry_msgs::PoseStamped new_path_pose = pose_msg;

	current_path.poses.push_back(new_path_pose);
	init = true;
}

void set_new_path(const nav_msgs::Path& new_path)
{
	current_path = new_path;
	
	goal_idx = 0;
	init = true;
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"carrot_follower");
	ros::NodeHandle node;
	
	ros::Subscriber pose_sub = node.subscribe("/pose", 1, update_pose);
	ros::Subscriber path_sub = node.subscribe("/path", 1, set_new_path);
	ros::Subscriber debug_sub = node.subscribe("/path_debug", 1, add_debug_goal);
	
	#ifdef SIMULATION
	current_path.poses.clear();
	goal_idx = 0;
	#endif
	
	carrot_pub = node.advertise<geometry_msgs::PoseStamped>("/carrot_pose", 1);
	
	ros::spin();
}
