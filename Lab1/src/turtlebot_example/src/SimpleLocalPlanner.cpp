#include "SimpleLocalPlanner.hpp"

#include <math.h>
#include "pose_util.hpp"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

#define TWIST_FWD(twist) (twist.linear.x)
#define TWIST_TURN(twist) (twist.angular.z)

SimpleLocalPlanner::SimpleLocalPlanner():
	goal_tolerance(0.1),
	traj_tolerance(5.0),
	fwd_rate(0.2),
	turn_rate(0.2),
	current_plan()
{ }

bool SimpleLocalPlanner::computeVelocityCommands(Twist &cmd_vel)
{
	if(current_plan.empty()) return false; //nowhere to go.
	
	const PoseStamped& current_goal = current_plan.front();
	bool close_enough = closeEnough(current_goal, latest_pose);
	double hdg_error = getHeadingError(current_goal, latest_pose);
	
	if (abs(hdg_error) > traj_tolerance)
	{
		TWIST_TURN(cmd_vel) = turn_rate * (hdg_error > 0? -1 : +1);
		if(close_enough) TWIST_FWD(cmd_vel) = 0.0; //stop forward movement
		return true;
	}
	
	if (!close_enough)
	{
		TWIST_FWD(cmd_vel) = fwd_rate; //drive forward
		TWIST_TURN(cmd_vel) = 0.0;
		return true;
	}
	
	//clear the current goal and re-evaluate
	finishedCurrentGoal();
	return computeVelocityCommands(cmd_vel);
}

void SimpleLocalPlanner::initialize(const PoseStamped& init_pose)
{
	latest_pose = init_pose;
}

bool SimpleLocalPlanner::isGoalReached()
{
	return current_plan.empty();
}

void SimpleLocalPlanner::updateLatestPose(const PoseWithCovarianceStamped& latest)
{
	latest_pose.pose = latest.pose.pose; //ROS is strange sometimes...
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

//Returns whether we are close enough to the current goal
bool SimpleLocalPlanner::closeEnough(const PoseStamped& goalpose, const PoseStamped& curpose)
{
	double dx = POSEST_X(goalpose) - POSEST_X(curpose);
	double dy = POSEST_Y(goalpose) - POSEST_Y(curpose);
	double dist = sqrt(pow(dx, 2) + pow(dy, 2));
	
	return dist <= this->goal_tolerance;
}

//Returns the heading difference between the direct line path to our goal, and the current pose.
double SimpleLocalPlanner::getHeadingError(const PoseStamped& goalpose, const PoseStamped& curpose)
{
	double dx = POSEST_X(goalpose) - POSEST_X(curpose);
	double dy = POSEST_Y(goalpose) - POSEST_Y(curpose);
	double hdg = atan2(dy, dx);
	double yaw = tf::getYaw(POSEST_O(curpose)); // Robot Yaw
	ROS_DEBUG("Yaw is: %f", yaw);
	
	return yaw - hdg;
}

//Clears the current goal and proceeds to the next one
void SimpleLocalPlanner::finishedCurrentGoal()
{
	if(!current_plan.empty()) current_plan.pop_front();
}