#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

/*
	Store and manipulate occupancy grids
*/

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

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
		int _wlen, _hlen;
		std::vector<logit_val> _grid_store;
		
		tf::Transform _origin, _center;
		tfScalar _grid_scale;
	
	public:
		OccupancyGrid(double w, double h, double cell_size, double center_x, double center_y);
		OccupancyGrid(double w, double h, double cell_size, const tf::Vector3& center_loc);
		int getWidth() const;
		int getHeight() const;
		tfScalar getScale() const; //The real-world width and height of each grid cell
		const tf::Transform& getOrigin() const;
		
		//Returns the transform from the map origin to the map center
		const tf::Transform& getCenter() const;
		
		tf::Vector3 toGridFrame(const tf::Vector3& vect) const;
		
		logit_val readValue(int i, int j) const;
		logit_val& valueAt(int i, int j);
		
		void writeToMsg(nav_msgs::OccupancyGrid& msg) const;
};

#include <ostream>

//debug print
std::ostream& operator<<(std::ostream& strm, const OccupancyGrid& grid);

#endif
