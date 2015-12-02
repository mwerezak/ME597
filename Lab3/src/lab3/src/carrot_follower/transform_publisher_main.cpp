#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle node;
	
	tf::TransformBroadcaster tf_bcaster;
	tf::Transform ident;
	ident.setIdentity();
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();   //Check for new messages
		
		tf_bcaster.sendTransform(tf::StampedTransform(ident, ros::Time::now(), "world", "odom"));
		
		loop_rate.sleep(); //Maintain the loop rate
	}
	
	return 0;
}