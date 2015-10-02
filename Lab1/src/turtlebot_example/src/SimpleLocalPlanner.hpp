#ifndef SIMPLELOCALPLANNER_HPP
#define SIMPLELOCALPLANNER_HPP

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

//A dumb local planner that drives the turtlebot straight at a goal,
//only stopping to correct if the angle off exceeds some tolerance.
//Mainly to get myself familiar with the ROS nav_core API.

//SimpleLocalPlanner expects all poses supplied to it to be in the same reference frame.
class SimpleLocalPlanner : nav_core::BaseLocalPlanner
{
	private:
		list<PoseStamped> current_plan;
		PoseStamped latest_pose;
		
		bool closeEnough(const PoseStamped& goalpose, const PoseStamped& curpose);
		double getHeadingError(const PoseStamped& goalpose, const PoseStamped& curpose);
		void finishedCurrentGoal();

	public:
		double goal_tolerance;
		double traj_tolerance;
		double fwd_rate;
		double turn_rate;
		
		SimpleLocalPlanner();
		
		void updateLatestPose(const PoseWithCovarianceStamped& latest_pose);
		
		//BaseLocalPlanner methods
		virtual bool computeVelocityCommands(Twist &cmd_vel);
		virtual void initialize(const PoseStamped& init_pose);
		virtual bool isGoalReached();
		virtual bool setPlan (const vector<PoseStamped> &plan);
};

#endif