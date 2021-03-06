#include "ray_tracing.h"

//copied from the example on LEARN
inline short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) 
{
	int dx = abs(x1 - x0);
	int dy = abs(y1 - y0);
	int dx2 = x1 - x0;
	int dy2 = y1 - y0;

	//overestimates the amount of memory we need, but guaranteed to avoid reallocations
	x.reserve(dx + 1);
	y.reserve(dy + 1);

	const bool s = abs(dy) > abs(dx);

	if (s) {
		int dx2 = dx;
		dx = dy;
		dy = dx2;
	}

	int inc1 = 2 * dy;
	int d = inc1 - dx;
	int inc2 = d - dx;

	x.push_back(x0);
	y.push_back(y0);

	while (x0 != x1 || y0 != y1) {
		if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
		if (d < 0) d += inc1;
		else {
			d += inc2;
			if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
		}

		//Add point to vector
		x.push_back(x0);
		y.push_back(y0);
	}
}

/*
	GridRayTrace Impl
*/

GridRayTrace::GridRayTrace(tf::Vector3 start, tf::Vector3 end, const OccupancyGrid& grid_ref)
	: _x_store(), _y_store()
{
	//Transform (x,y) into map frame
	start = grid_ref.toGridFrame(start);
	end = grid_ref.toGridFrame(end);
	
	double x0 = start.getX(), y0 = start.getY();
	double x1 = end.getX(), y1 = end.getY();
	
	//ROS_WARN("start: (%f, %f) -> end: (%f, %f)", x0, y0, x1, y1);
	
	bresenham
		(
			floor(x0), floor(y0), 
			floor(x1), floor(y1), 
			_x_store, _y_store
		);
	
	_cur_idx = 0;
	_max_idx = std::min(_x_store.size(), _y_store.size());
}

bool GridRayTrace::hasNext() const
{
	return _cur_idx < _max_idx;
}

//Writes the next i,j pair to the given pointers.
//Returns true if there is another i,j pair after this one, false if the ray trace is done.
bool GridRayTrace::getNextPoint(int& i, int& j)
{
	if(!hasNext()) return false;
	
	i = _x_store[_cur_idx];
	j = _y_store[_cur_idx];
	_cur_idx++;
	
	return true;
}
