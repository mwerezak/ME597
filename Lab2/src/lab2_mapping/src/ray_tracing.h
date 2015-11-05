#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <tf/tf.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include "occupancy_grid.h"

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y); //copied from example code

tf::Vector3 scaledTransform(tf::Vector3 vect, tf::Transform basis, tfScalar scale);

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
