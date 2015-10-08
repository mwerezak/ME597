#include "SimpleLocalPlanner.hpp"

#include <math.h>
#include <angles/angles.h>
#include <ros/time.h>
#include "pose_util.hpp"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

SimpleLocalPlanner::SimpleLocalPlanner():
	current_plan()
{ 
	goal_tolerance = 0.3;
	traj_tolerance = angles::from_degrees(10.0);
	fwd_rate = 0.2;
	turn_rate = 0.2;
}

bool SimpleLocalPlanner::computeVelocityCommands(Twist &cmd_vel)
{
	if(current_plan.empty()) return false; //nowhere to go
	
	double latest_pose_age = (Time::now() - latest_pose.header.stamp).toSec();
	const PoseStamped& current_goal = current_plan.front();
	
	double hdg_error = getHeadingError(current_goal, latest_pose);
	double distance = calcDistance(current_goal, latest_pose);
	bool close_enough = (distance <= goal_tolerance);
	ROS_WARN("BTT is: %+f, RTT is: %f", angles::to_degrees(hdg_error), distance);
	
	//*** Reached the goal
	if(close_enough)
	{
		//clear the current goal and re-evaluate
		finishedCurrentGoal();
		return computeVelocityCommands(cmd_vel);
	}
	
	//** Check for correct bearing to target
	
	//proportional control with saturation
	double prop_fwd_rate = min(distance*(0.5*fwd_rate/goal_tolerance), fwd_rate);
	if(abs(hdg_error) <= traj_tolerance)
	{
		TWIST_FWD(cmd_vel) = prop_fwd_rate; //drive forward
		TWIST_TURN(cmd_vel) = 0.0;
		return true;
	}
	
	if(abs(angles::from_degrees(180) - abs(hdg_error)) <= traj_tolerance)
	{
		TWIST_FWD(cmd_vel) = -prop_fwd_rate; //drive reverse
		TWIST_TURN(cmd_vel) = 0.0;
		return true;
	}
	
	//*** Not correct bearing, turn to target
	
	//far enough away we should aim to reverse
	if(hdg_error > angles::from_degrees(100))
	{
		hdg_error -= 180;
	}
	else if(hdg_error < -angles::from_degrees(100))
	{
		hdg_error += 180;
	}

	//proportional control with saturation
	double prop_turn_rate = -hdg_error*(0.5*turn_rate/traj_tolerance);
	prop_turn_rate = min(prop_turn_rate,  turn_rate); //saturation
	prop_turn_rate = max(prop_turn_rate, -turn_rate);
	
	TWIST_TURN(cmd_vel) = prop_turn_rate;
	TWIST_FWD(cmd_vel) = 0.0;
	return true;
}

void SimpleLocalPlanner::initialize(string, tf::TransformListener*, costmap_2d::Costmap2DROS*)
{
	return; //do nothing
}

bool SimpleLocalPlanner::isGoalReached()
{
	return current_plan.empty();
}

void SimpleLocalPlanner::updateLatestPose(const PoseWithCovarianceStamped& latest)
{
	latest_pose.pose = latest.pose.pose; //ROS can be strange
}

bool SimpleLocalPlanner::setPlan (const vector<PoseStamped> &plan)
{
	current_plan.clear();
	for(vector<PoseStamped>::const_iterator itr = plan.begin(); itr != plan.end(); itr++)
	{
		current_plan.push_back(*itr);
	}
	return true;
}

double SimpleLocalPlanner::calcDistance(const PoseStamped& pose_a, const PoseStamped& pose_b)
{
	double dx = POSEST_X(pose_a) - POSEST_X(pose_b);
	double dy = POSEST_Y(pose_a) - POSEST_Y(pose_b);
	double dist = sqrt(pow(dx, 2) + pow(dy, 2));
	
	return dist;
}

//Returns the heading difference between the direct line path to our goal, and the current pose.
double SimpleLocalPlanner::getHeadingError(const PoseStamped& goalpose, const PoseStamped& curpose)
{
	double dx = POSEST_X(goalpose) - POSEST_X(curpose);
	double dy = POSEST_Y(goalpose) - POSEST_Y(curpose);
	double hdg = atan2(dy, dx);
	double yaw = tf::getYaw(POSEST_O(curpose)); // Robot Yaw
	
	return angles::normalize_angle(yaw - hdg);
}

//Clears the current goal and proceeds to the next one
void SimpleLocalPlanner::finishedCurrentGoal()
{
	if(!current_plan.empty()) current_plan.pop_front();
}
