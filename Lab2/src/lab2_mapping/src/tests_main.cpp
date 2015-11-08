#include <ostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include "ray_tracing.h"
#include "occupancy_grid.h"
#include "laser_scan.h"

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

/*
	Tests Definitions
*/
void TestGridRayTrace(std::ostream& output)
{
	tf::Vector3 start(1.0, 1.0, 0.0);
	tf::Vector3 end(10.0, 10.0, 0.0);
	
	output << start.getX() << "," << start.getY();
	output << " -> ";
	output << end.getX() << "," << end.getY();
	output << "\n";
	
	OccupancyGrid test_grid(0, 0, 2.0, 0.0, 0.0);
	
	int i, j;
	GridRayTrace test_trace(start, end, test_grid);
	while(test_trace.getNextPoint(i, j))
		output << i << ", " << j << "\n";
}

void TestOccupancyGrid(std::ostream& output)
{
	tf::Vector3 origin(1.0, 1.0, 0.0);
	OccupancyGrid test_grid(5, 5, 2.0, 1.0, 1.0);
	test_grid.valueAt(0,0) = 0.5;
	test_grid.valueAt(2,3) = 0.7;
	output << test_grid;
}

void TestMappingUpdate(std::ostream& output)
{
	sensor_msgs::LaserScan test_scan;
	test_scan.angle_min = angles::from_degrees(-90.0);
	test_scan.angle_max = angles::from_degrees(+90.0);
	test_scan.angle_increment = angles::from_degrees(45.0);
	test_scan.range_min = 1.7;
	test_scan.range_max = 5.5;
	for(double angle = test_scan.angle_min; angle <= test_scan.angle_max; angle += test_scan.angle_increment)
	{
		test_scan.ranges.push_back(4.5);
	}
	
	tf::Vector3 loc(-5.0, 0.0, 0.0);
	tf::Quaternion rot;
	rot.setRPY(0.0, 0.0, angles::from_degrees(-90.0));
	tf::Transform robot(rot, loc);
	
	OccupancyGrid test_grid(12, 12, 1.0, 0.0, 0.0);
	MappingUpdate(test_grid, test_scan, robot);
	
	tf::Vector3 beam_start(0.0, 0.0, 0.0);
	tf::Vector3 beam_end(0.0, 6.0, 0.0);
	MapUpdateBeamHit(test_grid, beam_start, beam_end);
	
	output << test_grid;
}
 
int main(int argc, char** argv)
{
	std::ostream* output = &std::cout;
	
	/*
	std::fstream fout;

	if(argc > 1)
	{
		fout.open(argv[1]);
		output = &fout;
	}
	*/
	
	//*output << "\n*** TestGridRayTrace:\n";
	//TestGridRayTrace(*output);
	
	//*output << "\n*** TestOccupancyGrid:\n";
	//TestOccupancyGrid(*output);
	
	*output << "\n*** TestMappingUpdate:\n";
	TestMappingUpdate(*output);
	
	*output << "\n\n";
	//output.close();
	return 0;
}
