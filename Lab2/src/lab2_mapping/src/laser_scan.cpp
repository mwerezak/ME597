#include "laser_scan.h"

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include "ray_tracing.h"

static const logit_val OCCUPANCY_HIT_FEATURE = logit(0.9); //occupancy of a cell where the beam hit a feature
static const logit_val OCCUPANCY_HIT_AHEAD = logit(0.05); //occupancy of a cell where where the beam passed through and hit a feature behind
static const logit_val OCCUPANCY_NOTHING = logit(0.1); //occupancy of a cell where where the beam hit nothing at all

void MappingUpdate(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data)
{
	//TODO add transformation to world frame
	//For now just assume robot is at world origin with default orientation.
	
	/*
	From ROS docs:
		angles are measured around the positive Z axis (counterclockwise, if Z is up)
		with zero angle being forward along the x axis.
	*/
	const tf::Vector3 scan_basis(1.0, 0.0, 0.0); //unit vector corresponding to zero angle.
	const tf::Vector3 zaxis(0.0, 0.0, 1.0);
	
	tf::Transform pan_transform; //used to rotate the scan with angle
	
	//iterate over each beam
	double angle = scan_data.angle_min;
	int beam_idx = 0;
	while(angle <= scan_data.angle_max)
	{
		double beam_range = scan_data.ranges[beam_idx];
		
		//not sure if ranges[beam_idx] > range_max means nothing was hit or what.
		//ROS docs say to discard the values so I guess that's what we'll do.
		if(scan_data.range_min <= beam_range && beam_range <= scan_data.range_max)
		{
			tf::Quaternion pan_rotation(zaxis, angle);
			pan_transform.setRotation(pan_rotation);

			tf::Vector3 beam_start = pan_transform(scan_basis);
			beam_start *= scan_data.range_min;

			tf::Vector3 beam_end = pan_transform(scan_basis);
			beam_end *= scan_data.ranges[beam_idx];
			
			MapUpdateBeamHit(occ_map, beam_start, beam_end);
		}
		
		angle += scan_data.angle_increment;
		beam_idx++;
	}
}

//Updates an occupancy map by modifying cells along the beam using beamval and modifying the end-cell using endval.
void _mapUpdateTrace(OccupancyGrid& occ_map, GridRayTrace& trace, logit_val beamval, logit_val endval)
{
	int i, j;
	while(trace.getNextPoint(i, j))
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

//Updates the map given a beam that hit a feature at end
void MapUpdateBeamHit(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& hit_feature)
{
	GridRayTrace trace(beam_start, hit_feature, occ_map);
	_mapUpdateTrace(occ_map, trace, OCCUPANCY_HIT_AHEAD, OCCUPANCY_HIT_FEATURE);
}

//Updates the map given a beam that hit nothing
void MapUpdateBeamMiss(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& beam_end)
{
	GridRayTrace trace(beam_start, beam_end, occ_map);
	_mapUpdateTrace(occ_map, trace, OCCUPANCY_NOTHING, OCCUPANCY_NOTHING);
}