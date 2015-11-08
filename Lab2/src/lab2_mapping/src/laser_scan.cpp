#include "laser_scan.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include "frames.h"
#include "ray_tracing.h"

static const logit_val OCCUPANCY_HIT_FEATURE = logit(0.6); //occupancy of a cell where the beam hit a feature
static const logit_val OCCUPANCY_HIT_AHEAD = logit(0.4); //occupancy of a cell where where the beam passed through and hit a feature behind

void UpdateMapFromScan(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data)
{
	tf::StampedTransform robot_pos; //locate the robot
	ros::Time scan_time = scan_data.header.stamp;
	
	try
	{
		static tf::TransformListener tf_listener;
		
		//Block until a position is available at the time of the scan
		tf_listener.waitForTransform(WORLD_FRAME, ROBOT_FRAME, scan_time, ros::Duration(1.0));
		
		tf_listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, scan_time, robot_pos);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	
	MappingUpdate(occ_map, scan_data, robot_pos);
}

void MappingUpdate(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data, const tf::Transform& robot_pos)
{
	/*
	From ROS docs:
		angles are measured around the positive Z axis (counterclockwise, if Z is up)
		with zero angle being forward along the x axis.
	*/
	const tf::Vector3 scan_origin(0.0, 0.0, 0.0);
	const tf::Vector3 scan_basis(1.0, 0.0, 0.0); //unit vector corresponding to zero angle.
	
	#ifdef LIVE
	const tf::Vector3 zaxis(0.0, 0.0, -1.0);
	#else
	const tf::Vector3 zaxis(0.0, 0.0, 1.0);
	#endif
	
	tf::Transform pan_transform; //used to rotate the scan with angle
	pan_transform.setOrigin(scan_origin);
	
	//iterate over each beam
	double angle = scan_data.angle_min;
	for(int beam_idx = 0; beam_idx < scan_data.ranges.size(); beam_idx++)
	{
		double beam_range = scan_data.ranges[beam_idx];
		
		//not sure if ranges[beam_idx] > range_max means nothing was hit or what.
		//ROS docs say to discard the values so I guess that's what we'll do.
		if(!std::isnan(beam_range) && scan_data.range_min <= beam_range && beam_range <= scan_data.range_max)
		{
			tf::Quaternion pan_rotation(zaxis, angle);
			pan_transform.setRotation(pan_rotation);

			tf::Vector3 beam_start = pan_transform(scan_basis);
			beam_start *= scan_data.range_min;

			tf::Vector3 beam_end = pan_transform(scan_basis);
			beam_end *= scan_data.ranges[beam_idx];
			
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
			
			MapUpdateBeamHit(occ_map, robot_pos(beam_start), robot_pos(beam_end));
		}
		
		angle += scan_data.angle_increment;
	}
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
void MapUpdateBeamHit(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& hit_feature)
{
	GridRayTrace trace(beam_start, hit_feature, occ_map);
	_mapUpdateTrace(occ_map, trace, OCCUPANCY_HIT_AHEAD, OCCUPANCY_HIT_FEATURE);
}
