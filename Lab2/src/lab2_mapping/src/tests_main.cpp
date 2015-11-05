#include <fstream>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <angles/angles.h>
#include "ray_tracing.h"
#include "occupancy_grid.h"

void setTransformRPY
	(
		tf::Transform& transform,
		tfScalar x, tfScalar y, tfScalar z, 
		tfScalar roll, tfScalar pitch, tfScalar yaw
	)
{
	tf::Vector3 origin(x, y, z);
	transform.setOrigin(origin);
	
	tf::Quaternion rotation;
	rotation.setRPY(roll, pitch, yaw);
	transform.setRotation(rotation);
}

void setTransform2D(tf::Transform& transform, tfScalar x, tfScalar y, tfScalar yaw)
{
	setTransformRPY(transform, x, y, 0.0, 0.0, 0.0, yaw);
}

void TestGridRayTrace(std::ofstream& output)
{
	output << "*** GridRayTrace\n";
	
	tf::Vector3 start(2.0, 2.0, 0.0);
	tf::Vector3 end(+11.0, 13.0, 0.0);
	
	output << start.getX() << "," << start.getY();
	output << " -> ";
	output << end.getX() << "," << end.getY();
	output << "\n";
	
	tf::Transform origin;
	setTransform2D(origin, 1.0, 1.0, angles::from_degrees(90));
	
	OccupancyGrid test_grid(0, 0, 2.0, origin);
	
	int i, j;
	GridRayTrace test_trace(start, end, test_grid);
	while(test_trace.getNextPoint(i, j))
		output << i << ", " << j << "\n";
}

void TestOccupancyGrid(std::ofstream& output)
{
	tf::Transform origin;
	setTransform2D(origin, 1.0, 1.0, angles::from_degrees(90));
	OccupancyGrid test_grid(5, 5, 2.0, origin);
	test_grid.valueAt(0,0) = 0.5;
	test_grid.valueAt(2,3) = 0.7;
	output << test_grid;
}

int main()
{
	std::ofstream output;
	output.open("test_output.txt");
	
	output << "\nTestGridRayTrace:\n";
	TestGridRayTrace(output);
	
	output << "\nTestOccupancyGrid:\n";
	TestOccupancyGrid(output);
	
	output << "\n\n";
	output.close();
	return 0;
}
