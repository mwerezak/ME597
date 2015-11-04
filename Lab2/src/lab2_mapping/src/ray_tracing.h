#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <geometry_msgs/Pose.h>

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y); //copied from example code

class GridRayTrace
{
	public:
		//Constructor
		GridRayTrace(double x0, double y0, double x1, double y1, geometry_msgs::Pose map_origin);
		
		//Writes the next i,j pair to the given pointers.
		//Returns true if there is another i,j pair after this one, false if the ray trace is done.
		bool getNextPoint(int* i, int* j);
};

#endif
