#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <geometry_msgs/Pose.h>

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
