#include "occupancy_grid.h"

/*
OccupancyGrid::OccupancyGrid():
	_width(0), 
	_height(0) 
{ }
*/

OccupancyGrid::OccupancyGrid(int w, int h, tfScalar cell_size, tf::Transform& origin):
	_width(w), 
	_height(h), 
	_grid_scale(cell_size), 
	_grid_store(w*h, 0.0),
	_grid_origin(origin)
{
	_grid_origin_inv = _grid_origin.inverse();
}

int OccupancyGrid::getWidth() const { return _width; }
int OccupancyGrid::getHeight() const { return _height; }

const tf::Transform& OccupancyGrid::fromGridFrame() const { return _grid_origin; }
const tf::Transform& OccupancyGrid::toGridFrame() const { return _grid_origin_inv; }
tfScalar OccupancyGrid::getScale() const { return _grid_scale; }

logit_val OccupancyGrid::readValue(int i, int j) const 
{
	return _grid_store[i + j*_width];
}

logit_val& OccupancyGrid::valueAt(int i, int j)
{
	return _grid_store[i + j*_width];
}

std::ostream& operator<<(std::ostream& strm, const OccupancyGrid& grid) 
{
	strm << "[[\n";
	for(int j = grid.getHeight() - 1; j >= 0 ; j--)
	{
		strm << grid.readValue(0, j);
		for(int i = 1; i < grid.getWidth(); i++)
		{
			strm << ", " << grid.readValue(i, j);
		}
		strm << "\n";
	}
	strm << "]]";
}
