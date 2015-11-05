#include <fstream>
#include "ray_tracing.h"

int main()
{
	std::ofstream output;
	output.open("test_output.txt");
	
	//GridRayTrace
	output << "*** GridRayTrace\n";
	output << "-7,0 -> -11,13:\n";
	
	/*
	GridRayTrace test_trace(-7.0, 0.0, -11.0, 13.0);
	int i, j;
	while(test_trace.getNextPoint(i, j))
		output << i << ", " << j << "\n";
	*/
	
	output.close();
	return 0;
}
