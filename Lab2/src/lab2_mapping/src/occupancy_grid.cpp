#include "occupancy_grid.h"

#include <math.h>
#include <tf/transform_datatypes.h>

logit_val logit(prob_val p)
{
	return log(p/(1-p));
}

prob_val probability(logit_val logit)
{
	return exp(logit)/(1+exp(logit));
}

OccupancyGrid::OccupancyGrid(int w, int h, tfScalar cell_size, double x, double y):
	_wlen(ceil(w/cell_size)), 
	_hlen(ceil(h/cell_size)), 
	_grid_scale(cell_size), 
	_grid_store(w*h, 0.0)
{
	tf::Vector3 loc(x - (w/2.0), y - (h/2.0), 0.0);
	tf::Quaternion rotation;
	rotation.setRPY(0.0, 0.0, 0.0);
	
	tf::Transform origin(rotation, loc);
	_to_grid_frame = origin.inverse();
}

OccupancyGrid::OccupancyGrid(int w, int h, tfScalar cell_size, const tf::Vector3& origin_loc):
	_wlen(ceil(w/cell_size)), 
	_hlen(ceil(h/cell_size)), 
	_grid_scale(cell_size), 
	_grid_store(w*h, 0.0)
{
	tf::Vector3 loc(origin_loc.getX() - (w/2.0), origin_loc.getY() - (h/2.0), 0.0);
	tf::Quaternion rotation;
	rotation.setRPY(0.0, 0.0, 0.0);
	
	tf::Transform origin(rotation, loc);
	_to_grid_frame = origin.inverse();
}

int OccupancyGrid::getWidth() const { return _wlen; }
int OccupancyGrid::getHeight() const { return _hlen; }
tfScalar OccupancyGrid::getScale() const { return _grid_scale; }

tf::Vector3 OccupancyGrid::toGridFrame(const tf::Vector3& vect) const
{
	tf::Vector3 result = _to_grid_frame(vect);
	result /= _grid_scale;
	
	return result;
}


logit_val OccupancyGrid::readValue(int i, int j) const 
{
	return _grid_store[i + j*_wlen];
}

logit_val& OccupancyGrid::valueAt(int i, int j)
{
	return _grid_store[i + j*_wlen];
}

#include <iomanip>

std::ostream& operator<<(std::ostream& strm, const OccupancyGrid& grid) 
{
	std::ios state(NULL);
	state.copyfmt(strm);
	
	strm << "[[\n";
	strm << std::setw(4) << std::setprecision(2);
	strm << std::setiosflags(std::ios::showpoint | std::ios::showpos);
	for(int j = grid.getHeight() - 1; j >= 0 ; j--)
	{
		strm << grid.readValue(0, j);
		for(int i = 1; i < grid.getWidth(); i++)
		{
			strm << ", " << grid.readValue(i, j);
		}
		strm << "\n";
	}
	
	strm.copyfmt(state);
	
	strm << "]]";
}
