
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Point latest_carrot;
geometry_msgs::PoseStamped latest_pose;
ros::Publisher nav_publisher, pose_publisher;
bool init = false;

const double STOP_TOL = 0.1;
const double DRIVE_SPEED = 0.15;
const double TURN_TOL = angles::from_degrees(5.0);
const double PRE_TURN = angles::from_degrees(25.0);
const double TURN_SPEED = 0.35;
geometry_msgs::Twist simple_drive_control()
{
	tf::Transform robot;
	tf::Vector3 carrot;
	
	//find the carrot in the robot frame
	tf::poseMsgToTF(latest_pose.pose, robot);
	tf::pointMsgToTF(latest_carrot, carrot);
	
	tf::Vector3 goal = robot.inverse()(carrot);
	
	double bearing = atan2(goal.getY(), goal.getX()); //if forward is +x
	double range = goal.length();
	
	ROS_INFO("Bearing is: %fdeg, Range is: %f", angles::to_degrees(bearing), range);
	
	//TODO: scale PRE_TURN based on goal distance
	geometry_msgs::Twist cmd;
	if(range > STOP_TOL && fabs(bearing) < PRE_TURN)
	{ cmd.linear.x = DRIVE_SPEED; }
	else
	{ cmd.linear.x = 0.0; }
	
	if(fabs(bearing) > TURN_TOL)
	{ cmd.angular.z = TURN_SPEED*(bearing > 0.0? 1.0 : -1.0); }
	else
	{ cmd.angular.z = 0.0; }
	
	return cmd;
}

void UpdateCarrot(const geometry_msgs::PoseStamped& new_carrot)
{
	latest_carrot = new_carrot.pose.position;
	
	geometry_msgs::Point loc = new_carrot.pose.position;
	ROS_INFO("Recieved new carrot: (%f, %f, %f)", loc.x, loc.y, loc.z);
}

void UpdatePose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	latest_pose.header = msg.header;
	latest_pose.pose = msg.pose.pose;

	pose_publisher.publish(latest_pose);

	if(!init)
	{
		latest_carrot = latest_pose.pose.position;
		init = true;
	}
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"carrot_follower");
	ros::NodeHandle node;
	
	ros::Subscriber carrot_sub = node.subscribe("/carrot", 1, UpdateCarrot);
	ros::Subscriber pose_sub = node.subscribe("/robot_pos", 1, UpdatePose);
	
	nav_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/debug_pose", 1);
	
	ros::Rate loop_rate(20);    //20Hz update rate
	
	while (ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		
		if(init) 
		{
			nav_publisher.publish(simple_drive_control()); 
		}
		
		loop_rate.sleep(); //Maintain the loop rate
	}
}
