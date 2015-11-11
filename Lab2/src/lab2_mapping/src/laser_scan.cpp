#include "laser_scan.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include "frames.h"
#include "ray_tracing.h"

static const logit_val OCCUPANCY_HIT_FEATURE = logit(0.51); //occupancy of a cell where the beam hit a feature
static const logit_val OCCUPANCY_HIT_AHEAD = logit(0.49); //occupancy of a cell where where the beam passed through and hit a feature behind

inline tf::Transform interpolateTF(tf::Transform start, tf::Transform end, double ratio)
{
	tf::Vector3 lerp_pos = start.getOrigin().lerp(end.getOrigin(), ratio);
	tf::Quaternion lerp_rot = start.getRotation().slerp(end.getRotation(), ratio);
	
	return tf::Transform(lerp_rot, lerp_pos);
}

void UpdateMapFromScan(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data)
{
	ros::Time scan_start_time = scan_data.header.stamp;
	ros::Duration scan_duration = ros::Duration().fromSec(scan_data.time_increment * scan_data.ranges.size());
	ros::Time scan_end_time = scan_start_time + scan_duration;
	
	tf::StampedTransform robot_start_pos;
	tf::StampedTransform robot_end_pos;
	
	try
	{
		static tf::TransformListener tf_listener;
		
		//Block until a position is available at the time of the scan
		tf_listener.waitForTransform(WORLD_FRAME, ROBOT_FRAME, scan_end_time + TIME_SHIFT, ros::Duration(1.0));
		
		tf_listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, scan_start_time + TIME_SHIFT, robot_start_pos);
		tf_listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, scan_end_time + TIME_SHIFT, robot_end_pos);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	
	//iterate over each beam
	for(int beam_idx = 0; beam_idx < scan_data.ranges.size(); beam_idx++)
	{
		double beam_range = scan_data.ranges[beam_idx];
		double angle = scan_data.angle_min + scan_data.angle_increment*beam_idx;
		
		if(_checkValidBeam(beam_idx, scan_data))
		{
			tf::Vector3 beam_start = tf::Vector3(0.0, 0.0, 0.0);
			tf::Vector3 beam_end = _getBeamHitPos(beam_range, angle);
			
			double ratio = beam_idx/double(scan_data.ranges.size() - 1);
			tf::Transform robot_pos = interpolateTF(robot_start_pos, robot_end_pos, ratio);
			
			_mapUpdateBeamHit(occ_map, robot_pos(beam_start), robot_pos(beam_end));
		}
	}
}

void _mappingUpdate(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data, const tf::Transform& robot_pos)
{
	//iterate over each beam
	double angle = scan_data.angle_min;
	for(int beam_idx = 0; beam_idx < scan_data.ranges.size(); beam_idx++)
	{
		double beam_range = scan_data.ranges[beam_idx];
		
		if(_checkValidBeam(beam_idx, scan_data))
		{
			tf::Vector3 beam_start = tf::Vector3(0.0, 0.0, 0.0);
			tf::Vector3 beam_end = _getBeamHitPos(beam_range, angle);
			
			/*
			ROS_WARN("angle: %fdeg\nbeam_start: (%f, %f)\nbeam_end: (%f, %f)", 
						angles::to_degrees(angle), 
						beam_start.getX(), beam_start.getY(),
						beam_end.getX(), beam_end.getY()
					);
			
			ROS_WARN("robot_pos: (%f, %f, %fdeg)",
						robot_pos.getOrigin().getX(),
						robot_pos.getOrigin().getY(),
						angles::to_degrees(tf::getYaw(robot_pos.getRotation()))
					);
			*/
			
			_mapUpdateBeamHit(occ_map, robot_pos(beam_start), robot_pos(beam_end));
		}
		
		angle += scan_data.angle_increment;
	}
}

bool _checkValidBeam(int beam_idx, const sensor_msgs::LaserScan& scan_data)
{
	double beam_range = scan_data.ranges[beam_idx];
	return (!std::isnan(beam_range) && scan_data.range_min <= beam_range && beam_range <= scan_data.range_max);
}

/*
From ROS docs:
	angles are measured around the positive Z axis (counterclockwise, if Z is up)
	with zero angle being forward along the x axis.
*/
tf::Vector3 _getBeamHitPos(double beam_range, double angle)
{
	const tf::Vector3 scan_basis(1.0, 0.0, 0.0); //unit vector corresponding to zero angle.
	
	const tf::Vector3 zaxis(0.0, 0.0, 1.0);
	
	tf::Transform pan_transform(tf::Quaternion(zaxis, angle), tf::Vector3(0.0, 0.0, 0.0)); 

	tf::Vector3 hit_pos = pan_transform(scan_basis);
	hit_pos *= beam_range;
	
	return hit_pos;
}

//Updates an occupancy map by modifying cells along the beam using beamval and modifying the end-cell using endval.
void _mapUpdateTrace(OccupancyGrid& occ_map, GridRayTrace& trace, logit_val beamval, logit_val endval)
{
	int i, j;
	while(trace.getNextPoint(i, j))
	{
		//ROS_WARN("%d, %d", i, j);
		if(0 <= i && i < occ_map.getWidth() && 0 <= j && j < occ_map.getHeight())
		{
			if(trace.hasNext())
			{
				occ_map.valueAt(i, j) += beamval;
			}
			else
			{
				occ_map.valueAt(i, j) += endval;
			}
		}
	}
}

//Updates the map given a beam that hit a feature at end
void _mapUpdateBeamHit(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& hit_feature)
{
	GridRayTrace trace(beam_start, hit_feature, occ_map);
	_mapUpdateTrace(occ_map, trace, OCCUPANCY_HIT_AHEAD, OCCUPANCY_HIT_FEATURE);
}
