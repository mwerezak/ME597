#include <fstream>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include "ray_tracing.h"


int main()
{
	std::ofstream output;
	output.open("test_output.txt");
	
	//GridRayTrace
	output << "*** GridRayTrace\n";
	
	tf::Vector3 start(2.0, 2.0, 0.0);
	tf::Vector3 end(+11.0, 13.0, 0.0);
	
	output << start.getX() << "," << start.getY();
	output << " -> ";
	output << end.getX() << "," << end.getY();
	output << "\n";
	
	geometry_msgs::Pose pose;
	OccupancyGrid test_grid(0, 0, pose);
	
	int i, j;
	GridRayTrace test_trace(start, end, test_grid);
	while(test_trace.getNextPoint(i, j))
		output << i << ", " << j << "\n";
	
	output.close();
	return 0;
}
