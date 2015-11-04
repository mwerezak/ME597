#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

/*
	Store and manipulate occupancy grids
*/

#include <geometry_msgs/Pose.h>

using namespace geometry_msgs;

typedef prob_val double;
typedef logit_val double;

//Converts a probability to a log odds ratio.
logit_val logit(prob_val probability);

//Converts a log odds ratio to probability.
prob_val probability(logit_val logit);


class OccupancyGrid
{
	private:
		int width, height;
		double* grid_store;
		Pose map_origin; 
	public:
		OccupancyGrid(int w, int h, Pose& origin);
		int getWidth() const;
		int getHeight() const;
		const Pose getOrigin() const;
		
		logit_val readValue(int i, int j) const;
		logit_val& valueAt(int i, int j);
};

#endif
