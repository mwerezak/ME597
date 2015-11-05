#include "occupancy_grid.h"

//Some dummy definitions so the linker won't complain.
const tf::Transform& OccupancyGrid::fromGridFrame() const
{
	static tf::Transform dummy;
	return dummy;
}

const tf::Transform& OccupancyGrid::toGridFrame() const
{
	static tf::Transform dummy;
	return dummy;
}

tfScalar OccupancyGrid::getScale() const
{ 
	return 0.0;
}
