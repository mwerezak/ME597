//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

//TODO: IMPROVE SQUARE ACCURACY -- MOVE SLOWER? GET BETTER MAP / AMCL DATA
//							 	-- INCREASE AMCL POSE DATA PUBLISH RATE

struct pose_data_t
{
public:

	double X;
	double Y;
	double Yaw;

	void copy_vals(pose_data_t);
};

void pose_data_t::copy_vals(pose_data_t in)
{
	X = in.X;
	Y = in.Y;
	Yaw = in.Yaw;
}		

//volatile 
pose_data_t cur_pose = {0, 0, 0};
const double square_size = 0.3;
const double linear_v = 0.2;
const double angular_v = 0.2;


//Callback function for the Position topic 
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	//This function is called when a new pose message is received

	cur_pose.X = msg.pose.pose.position.x; // Robot X position
	cur_pose.Y = msg.pose.pose.position.y; // Robot Y position
	cur_pose.Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_INFO("\npose_callback X: %f Y: %f Yaw: %f\n", cur_pose.X, cur_pose.Y, cur_pose.Yaw);
}

bool compare_pose (pose_data_t cur_pose, pose_data_t target_pose)
{
	return (cur_pose.X - target_pose.X < 0.1) && (cur_pose.Y - target_pose.Y < 0.1) 
			&& (cur_pose.Yaw - target_pose.Yaw < 0.1);
}

double calc_dist_travelled (pose_data_t start_pos, pose_data_t cur_pos)
{
	return ( sqrt( (start_pos.X - cur_pos.X)*(start_pos.X - cur_pos.X) + 
				(start_pos.Y- cur_pos.Y)*(start_pos.Y - cur_pos.Y) ) );
}

double increment_angle(double ang)
{
	ang += M_PI / 2;
	if (ang > M_PI){
		ang -= 2*M_PI;  //angle goes from -pi to pi
						//If angle > pi, should roll to -pi + (ang - pi) = ang - 2*pi
	}
	return ang;
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;

	//Subscribe to the desired topics and assign callbacks
	ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

	//Setup topics to Publish from this node
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    
	//Velocity control variable
	geometry_msgs::Twist vel;

	unsigned edge_num = 0; //which edge are we driving
	bool turning = false;
	int vel_sign = 1; //chifts sign of movement for edges 2 and 3
	pose_data_t start_pose = {0,0,0};
	double target_angle;

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

    
	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
    
		//Main loop code goes here:

		if (edge_num == 0){
			while (cur_pose.X == 0); //wait for first pose update
			edge_num++;	
			start_pose = cur_pose;
		}

		if (!turning)
		{
			if(calc_dist_travelled(start_pose, cur_pose) < square_size) // not where we want to be
			{
				vel.linear.x = linear_v;
			}
			else //where we want to be
			{
				vel.linear.x = vel.linear.y = vel.angular.z = 0;
				target_angle = increment_angle(cur_pose.Yaw);
				turning = true;
			}
		}
		else // turning
		{
			if(fabs(cur_pose.Yaw - target_angle) > 0.1)
			{
				 vel.angular.z = angular_v;
			}
			else //done turn
			{
				vel.linear.x = vel.linear.y = vel.angular.z = 0;
				turning = false;
				edge_num++;
				start_pose = cur_pose;
			}
		}

		velocity_publisher.publish(vel); // Publish the command velocity
		ROS_DEBUG("Main - Velocity commands: v - %f, w - %f", vel.linear.x, vel.angular.z);
		//ROS_INFO("current pose data in main loop X: %f Y: %f Yaw: %f", cur_pose.X, cur_pose.Y, cur_pose.Yaw);
		//ROS_INFO("start pose data in main loop X: %f Y: %f Yaw: %f", start_pose.X, start_pose.Y, start_pose.Yaw);
		
		if (!turning)
		{
			ROS_INFO("Have travelled %f out of %f", calc_dist_travelled(start_pose, cur_pose), square_size);
		}
		else{
			ROS_INFO("angle, target_angle, difference: %f, %f, %f", cur_pose.Yaw, target_angle, fabs(cur_pose.Yaw - target_angle) );
		}
 
	}

	return 0;
}
