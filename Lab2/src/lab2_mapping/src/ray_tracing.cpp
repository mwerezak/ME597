#include "ray_tracing.h"

//copied from the example on LEARN
inline short sgn(int x) { return x >= 0 ? 1 : -1; }

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) 
{
	int dx = abs(x1 - x0);
	int dy = abs(y1 - y0);
	int dx2 = x1 - x0;
	int dy2 = y1 - y0;

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

GridRayTrace::GridRayTrace
	(
		double x0, double y0, 
		double x1, double y1 
		//const OccupancyGrid& grid_ref
	)	
	: _x_store(), _y_store()
{
	//Transform (x,y) into map frame
	
	
	//overestimates the amount of memory we need, but guaranteed to avoid reallocations
	_x_store.reserve(ceil(abs(x1 - x0)));
	_y_store.reserve(ceil(abs(y1 - y0)));
	
	bresenham(x0, y0, x1, y1, _x_store, _y_store);
	_cur_idx = 0;
	_max_idx = std::min(_x_store.size(), _y_store.size());
}

//Writes the next i,j pair to the given pointers.
//Returns true if there is another i,j pair after this one, false if the ray trace is done.
bool GridRayTrace::getNextPoint(int& i, int& j)
{
	if(_cur_idx >= _max_idx)
		return false;
	
	i = _x_store[_cur_idx];
	j = _y_store[_cur_idx];
	_cur_idx++;
	
	return true;
}