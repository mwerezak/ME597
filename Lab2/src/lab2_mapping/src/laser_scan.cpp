#include "laser_scan.h"

#include "ray_tracing.h"

static const logit_val OCCUPANCY_HIT_FEATURE = logit(0.9); //occupancy of a cell where the beam hit a feature
static const logit_val OCCUPANCY_HIT_AHEAD = logit(0.05); //occupancy of a cell where where the beam passed through and hit a feature behind
static const logit_val OCCUPANCY_NOTHING = logit(0.1); //occupancy of a cell where where the beam hit nothing at all

void MappingUpdate(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data)
{
	//TODO add transformation to world frame
	//For now just assume robot is at world origin with default orientation.
	
	//iterate over each beam
	for(int angle = scan_data.angle_min; angle <= scan_data.angle_max; angle += scan_data.angle_increment)
	{
		//TODO
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
			occ_map.valueAt(i, j) = beamval;
		}
		else
		{
			occ_map.valueAt(i, j) = endval;
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