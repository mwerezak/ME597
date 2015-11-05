#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <tf/tf.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include "occupancy_grid.h"

class GridRayTrace
{
	private:
		std::vector<int> _x_store, _y_store;
		int _cur_idx, _max_idx;
	public:
		//Constructor
		GridRayTrace(tf::Vector3 start, tf::Vector3 end, const OccupancyGrid& grid_ref);
		
		//Writes the next i,j pair to the given pointers and returns true.
		//Returns false if there were no more points and i,j left untouched.
		bool getNextPoint(int& i, int& j);
};

#endif
