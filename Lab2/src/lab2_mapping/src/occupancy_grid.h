#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

/*
	Store and manipulate occupancy grids
*/

#include <tf/tf.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>

using namespace geometry_msgs;

typedef double prob_val;
typedef double logit_val;

//Converts a probability to a log odds ratio.
logit_val logit(prob_val probability);

//Converts a log odds ratio to probability.
prob_val probability(logit_val logit);


class OccupancyGrid
{
	private:
		int _width, _height;
		double* _grid_store; //ptr to 2D array
		
		tf::Transform _grid_origin, _grid_origin_inv;
		tfScalar _grid_scale;
	
	public:
		OccupancyGrid(int w, int h, Pose& origin);
		int getWidth() const;
		int getHeight() const;
		
		//Gets the transform representing the frame of the occupancy grid wrt the world.
		const tf::Transform& fromGridFrame() const;
		const tf::Transform& toGridFrame() const;
		tfScalar getScale() const; //The real-world width and height of each grid cell
		
		logit_val readValue(int i, int j) const;
		logit_val& valueAt(int i, int j);
};

#endif
