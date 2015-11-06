#include "occupancy_grid.h"

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

logit_val logit(prob_val p)
{
	return log(p/(1-p));
}

prob_val probability(logit_val logit)
{
	return exp(logit)/(1+exp(logit));
}

OccupancyGrid::OccupancyGrid(double w, double h, double cell_size, double x, double y):
	_wlen(ceil(w/cell_size)), 
	_hlen(ceil(h/cell_size)), 
	_grid_scale(cell_size), 
	_grid_store(_wlen*_hlen, 0.0)
{
	_origin.setOrigin(tf::Vector3(x - (w/2.0), y - (h/2.0), 0.0));
	_origin.setRotation(tf::createQuaternionFromYaw(0.0));
}

OccupancyGrid::OccupancyGrid(double w, double h, double cell_size, const tf::Vector3& origin_loc):
	_wlen(ceil(w/cell_size)), 
	_hlen(ceil(h/cell_size)), 
	_grid_scale(cell_size), 
	_grid_store(_wlen*_hlen, 0.0)
{
	_origin.setOrigin(tf::Vector3(origin_loc.getX() - (w/2.0), origin_loc.getY() - (h/2.0), 0.0));
	_origin.setRotation(tf::createQuaternionFromYaw(0.0));
}

int OccupancyGrid::getWidth() const { return _wlen; }
int OccupancyGrid::getHeight() const { return _hlen; }
tfScalar OccupancyGrid::getScale() const { return _grid_scale; }
const tf::Transform& OccupancyGrid::getOrigin() const { return _origin; }

tf::Vector3 OccupancyGrid::toGridFrame(const tf::Vector3& vect) const
{
	tf::Vector3 result = _origin.inverse()(vect);
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

//If needed: cache the message data and only write updates
void OccupancyGrid::writeToMsg(nav_msgs::OccupancyGrid& msg) const
{
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";
	
	msg.info.map_load_time = ros::Time::now();
	msg.info.resolution = _grid_scale;
	msg.info.width = getWidth();
	msg.info.height = getHeight();
	tf::poseTFToMsg(_origin, msg.info.origin);
	
	msg.data.resize(_grid_store.size());
	for(int i = 0; i < _grid_store.size(); i++)
	{
		char cell_data;
		if(_grid_store[i] == 0)
		{
			msg.data[i] = -1;
		}
		else
		{
			msg.data[i] = probability(_grid_store[i])*100;
		}
	}
	
}