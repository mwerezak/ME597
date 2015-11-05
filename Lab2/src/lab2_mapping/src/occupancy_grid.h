#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

/*
	Store and manipulate occupancy grids
*/

#include <tf/tf.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>

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
		std::vector<logit_val> _grid_store;
		
		tf::Transform _to_grid_frame;
		tfScalar _grid_scale;
	
	public:
		OccupancyGrid(int w, int h, tfScalar cell_size, tf::Transform& origin);
		int getWidth() const;
		int getHeight() const;
		tfScalar getScale() const; //The real-world width and height of each grid cell	
		
		tf::Vector3 toGridFrame(const tf::Vector3& vect) const;
		
		logit_val readValue(int i, int j) const;
		logit_val& valueAt(int i, int j);
};

#include <ostream>

//debug print
std::ostream& operator<<(std::ostream& strm, const OccupancyGrid& grid);

#endif
