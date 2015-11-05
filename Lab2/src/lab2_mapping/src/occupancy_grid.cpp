#include "occupancy_grid.h"

#include <angles/angles.h>
#include <tf/LinearMath/Quaternion.h>

//Some dummy definitions so the linker won't complain.
OccupancyGrid::OccupancyGrid(int w, int h, Pose& origin)
{}

const tf::Transform& OccupancyGrid::toGridFrame() const
{
	static tf::Transform dummy;
	
	tf::Vector3 origin(1.0, 1.0, 0.0);
	tf::Vector3 zaxis(0.0, 0.0, 1.0);
	
	tf::Quaternion rotation(zaxis, angles::from_degrees(90.0));
	
	dummy.setOrigin(origin);
	dummy.setRotation(rotation);
	dummy = dummy.inverse();
	
	return dummy;
}

tfScalar OccupancyGrid::getScale() const
{ 
	return 2.0;
}
