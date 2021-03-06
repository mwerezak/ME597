API references for ROS messages should be in the /docs folder.

### 1. Ray Tracing
Implement the bresenham line algorithm (just copy it from the course notes if needed).
Take start and end coordinates (in the world frame), a Pose indicating the location of the map origin.
Transform the start and end coordinates into the map frame, and use bresenham to produce a sequence of i,j indices indicating the occupancy grid bins along the line path.

##### Proposed interface:
I'd like us to use this iterator pattern for the lab since it's completely flexible while also being dead simple. It's used pretty much everywhere for a reason.

Internally you can do whatever you want. If you want to copy the bresenham algorithm from the course notes and just read from an internal array inside getNextPoint() that works. It's just that having this interface allows us to work independently and it's pretty simple while being flexible enough that we can do anything with the implementation later if we need.

```
class GridRayTrace
{
	public:
		//Constructor
		GridRayTrace(int x0, int y0, int x0, y0, geometry_msgs::Pose map_origin);
		
		//Writes the next i,j pair to the given pointers.
		//Returns true if there is another i,j pair after this one, false if the ray trace is done.
		bool getNextPoint(int* i, int* j);
};
```

### 2. Occupancy Map Update
Take a `sensor_msgs::LaserScan` and interpret it, pass arguments to `GridRayTrace`.
Uses the ray trace results to update the occupancy map according to an inverse measurement model.

### 3. Visualization
Read the occupancy map and publish `nav_msgs::OccupancyGrid` messages.
Ensure that RViz can see the messages and displays the visualization correctly.
Create some dummy occupancy maps for testing if needed.

### 4. System Integration
Glue everything together. Create the main ROS node, initialization, publishers, subscribers. Create launch scripts for demoing everything.
