/*
	Store and manipulate occupancy grids
*/

//Converts a probability to a log odds ratio.
double logit(double probability);

//Converts a log odds ratio to probability.
double probability(double logit);


class OccupancyGrid
{
	private:
		int width, height;
		double* grid_store;
		geometry_msgs::Pose map_origin; 
	public:
		OccupancyGrid(int w, int h);
		int getWidth() const;
		int getHeight() const;
		double readValue(int x, int y) const;
		double& valueAt(int x, int y);		
};


