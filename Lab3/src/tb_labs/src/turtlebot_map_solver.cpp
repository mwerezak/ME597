//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains the particle filter implementation for 
// Group 3
// Author: Daniel Murphy 
// With credit for sample code to: James Servos, Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <random>
#include <tf/transform_broadcaster.h>

//#define SIMULATION //comment out when running on turtlebot

//Constants
static const unsigned nM = 200; //Number of milestones
static const double dt = 0.05; //20 Hz
static double cell_size = 0.1;
static const int occ_thresh = 50; //Threshold for obstical presense: all 0s and 100s anyway

//Shared variables
static bool new_ips = false;
static bool vals_init;
static std::vector<geometry_msgs::Point> milestones;
static ros::Publisher pose_pub;
static double true_x, true_y, true_yaw;

//global frame measurements of velocity from odometry data
double odom_vx = 0;
double odom_vy = 0;
double odom_w = 0;

//global frame estimates of position based on odometry data
double odom_x = 0;
double odom_y = 0;
double odom_yaw = 0;

double cmd_vel_x = 0;
double cmd_vel_y = 0;
double cmd_vel_w = 0;

//Location of navigation goal
double goal_x = 0;
double goal_y = 0;

struct OccGrid
{
    std::vector<int8_t> map_data;
    unsigned width;
    unsigned height;
    double orig_x;
    double orig_y;
    bool init = false;

    double valAt(unsigned x_get, unsigned y_get) const
    {
        if (x_get >= 0 && y_get >= 0 && x_get < width && y_get < height)
            return map_data[x_get*width + y_get]; //assuming row major indexing
        else
            return -1;
    }
} map;

struct GraphNode
{
    int id;         //Node ID for checking if already in open set
    int x;          //occupancy grid index in x
    int y;          //occupancy grid index in y
    std::vector<GraphNode*> edges;   //Nearest edges
    std::vector<double> edge_lens;
    GraphNode * last_edge = NULL;   //last edge when traversing graph
    double cost = 0;                //Cost from start to here
    double cost_lb = 0;             //lower bound on cost
};

static std::vector <GraphNode> nodes;
static std::vector <GraphNode> spath;

struct EdgeLength //For associating edge lengths with indices
{
    int ind;
    double length;
};

void test_occ_grid(const OccGrid& grid)
{
    ROS_DEBUG("Occupancy grid implementation test");
    ROS_DEBUG("Width: %d, Height %d", grid.width, grid.height);
    ROS_DEBUG("Checking corner values");
    ROS_DEBUG("Checking 0, 0, value = %f", grid.valAt(0,0));
    ROS_DEBUG("Checking 0, %d, value = %f", grid.width - 1, grid.valAt(grid.width - 1, 0));
    ROS_DEBUG("Checking %d, 0, value = %f", grid.height - 1, grid.valAt(0,grid.height-1));
    ROS_DEBUG("Checking %d, %d, value = %f", grid.width - 1, grid.height - 1, 
        grid.valAt(grid.width - 1,grid.height-1));
    ROS_DEBUG("Occupancy grid seems OK to me!");
}

void init()
{
    // get starting position for the odom
    // has the benefit of not needing transforms for Aodom
    odom_x = true_x;
    odom_y = true_y;
    odom_yaw = true_yaw;


    ROS_DEBUG("init with global pts\n: X %f Y: %f Yaw: %f", 
                    odom_x, odom_y, odom_yaw);
    
    //initialize particle filter

    vals_init = true;
}

//Callback function for the Position topic (SIMULATION)
#ifdef SIMULATION
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{   
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    true_x = msg.pose[i].position.x;
    true_y = msg.pose[i].position.y;
    true_yaw = tf::getYaw(msg.pose[i].orientation);
    
    if(!vals_init)
        init();

    //ROS_DEBUG("ips callback X: %f Y: %f Yaw: %f", true_y, ips_y, true_yaw);
    new_ips = true;
}

#else
//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    const double scaling = 2.17;

    true_x = -scaling * msg.pose.pose.position.x; // Robot X psotition
    true_y = scaling * msg.pose.pose.position.y;  // Robot Y psotition
    true_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    if(!vals_init)
        init();

    //ROS_DEBUG("ips callback X: %f Y: %f Yaw: %f", true_x, true_y, true_yaw);
    new_ips = true;
}
#endif

void odom_callback(const nav_msgs::Odometry& msg)
{
    if (!vals_init) 
        return;

    odom_vx = msg.twist.twist.linear.x;
    odom_vy = msg.twist.twist.linear.y;
    odom_w = msg.twist.twist.angular.z;

    //zero order time integration, I don't think I need this for the map solver
    #ifdef SIMULATION
    odom_yaw = odom_yaw + odom_w * dt;
    #else
    odom_yaw = odom_yaw - odom_w * dt;
    #endif

    if (odom_yaw > (M_PI))
        odom_yaw -= 2*M_PI;

    odom_x = odom_x + odom_vx * cos(odom_yaw) * dt;
    odom_y = odom_y + odom_vx * sin(odom_yaw) * dt;

    // ROS_DEBUG("odom vel x: %f y: %f omega: %f", 
    //    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z);
    // ROS_DEBUG("odom dt = %f", dt); 
    // ROS_DEBUG("odom pos est x: %f y: %f yaw: %f\n", 
    //    odom_x, odom_y, odom_yaw);
}

void vel_callback(const  geometry_msgs::Twist& msg)
{
    //Get command signals when input by teleop
    cmd_vel_x = msg.linear.x;
    cmd_vel_y = msg.linear.y;
    cmd_vel_w = msg.angular.z;

    //ROS_DEBUG("vel_callback vx: %f, vy %f, w %f", cmd_vel_x, cmd_vel_y, cmd_vel_w);
}

double normal_pdf(double x, double m, double s)
{
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
    //Above code from:
    //http://stackoverflow.com/questions/10847007/using-the-gaussian-probability-density-function-in-c
}

static inline short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) 
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

bool check_collisions(int x0, int y0, int x1, int y1, OccGrid map, int pad_size)
{
    //Return false for collisions, true if not
    //ROS_DEBUG("Start coll check bw (%d, %d) and (%d, %d)", x0, y0, x1, y1);
    std::vector <int> linex, liney;

    bresenham(x0, y0, x1, y1, linex, liney);

    if(linex.size() == 0)
    {
        ROS_DEBUG("zero size vector from Bresenham");
        return false;
    }

    for(int i = 0; i < linex.size(); i++)
    {
        //Again HIGHLY sub-obtimal, but gets the job done for small pad size
        bool near_obst = false;
        for (int j = -1*pad_size; j < pad_size; j++)
        {
            for(int k = -1*pad_size; k < pad_size; k++)
            {
                int map_val = map.valAt(linex[i] + j, liney[i] + k);
                if (  map_val > occ_thresh || map_val == -1)
                {
                    near_obst = true;
                    break;
                }
            }
        }

        if(near_obst)
        {   
            //ROS_DEBUG("Collision at %d, %d, val %f", linex[i], liney[i], map.valAt(linex[i], liney[i]));
            return false;
        }
    }

    //ROS_DEBUG("End coll check, success");
    return true;
}

static inline bool edge_cmp(EdgeLength a, EdgeLength b)
{
    return (a.length < b.length); 
}

//Compute edge lengths, find nearest neighbors, link graph
std::vector <GraphNode> compute_edges(std::vector <geometry_msgs::Point> points, 
                            geometry_msgs::Point start_pt, geometry_msgs::Point end_pt, OccGrid map)
{
    static const unsigned nEdges = 15; //Number of edges between nodes;
    static const int pad_size = 2;
    std::vector <GraphNode> nodes;
    GraphNode cur, end;
    double dx, dy;

    ROS_DEBUG("Computing edges");

    if (points.size() == 0 || map.map_data.size() ==0)
    {
        //return empty graph if vectors unititialized
        return nodes;
    }

    cur.x = start_pt.x; //Start with start node (obviously)
    cur.y = start_pt.y;
    cur.id = 0;
    nodes.push_back(cur);

    for (int i = 0; i < points.size(); i++)
    {  
        cur.id = i + 1;
        cur.x = points[i].x;
        cur.y = points[i].y;
        dx = cur.x - end.x;
        dy = cur.y - end.y;
        cur.cost_lb = sqrt(dx*dx + dy*dy); //Lower bound on cost is distance to end
        nodes.push_back(cur);
    }

    end.x = end_pt.x;
    end.y = end_pt.y;
    end.id = nodes.size();

    nodes.push_back(end);

    //Algorithm
    //compute edge lengths, sort
    //Choose nEdges nearest neighbors, this builds graph
    double edge_len = 0;
    EdgeLength e;
    std::vector <EdgeLength> edge_lens;
    for (int i = 0; i < nodes.size(); i++)
    {
        edge_lens.clear();
        //ROS_DEBUG("Computing edges for node %d", i);
        
        //Compute all edge lengths for current node
        for(int j = 0; j < nodes.size(); j++)
        {
            if (nodes[j].id != nodes[i].id) //Don't compute edge to self
            {
                dx = nodes[i].x - nodes[j].x;
                dy = nodes[i].y - nodes[j].y;
                e.length = sqrt(dx*dx + dy*dy);
                e.ind = j;
                edge_lens.push_back(e);
                //ROS_DEBUG("Edge bw (%d, %d) and (%d, %d) has length %f", nodes[i].x, nodes[i].y,
                //                                        nodes[j].x, nodes[j].y, e.length);
            }
        }
        //ROS_DEBUG("Computed %lu edge lengths", edge_lens.size());
        std::sort(edge_lens.begin(), edge_lens.end(), edge_cmp);

        //Find nEdges nearest neighbors in edge_lens for ith node that don't have collisions
        int k = 0;
        int good_edges = 0;
        //ROS_DEBUG("Finding good edges for node %d", i);

        while( (good_edges < nEdges) && (k < edge_lens.size()) )
        {
            int edge_ind = edge_lens[k].ind; //Get node associated with edge length
            if ( check_collisions(nodes[edge_ind].x, nodes[edge_ind].y, 
                                    nodes[i].x, nodes[i].y, map, pad_size) )//No collision
            {         
                nodes[i].edges.push_back(&nodes[edge_ind]); //Put that node in the current node's edges
                nodes[i].edge_lens.push_back(edge_lens[k].length);
                good_edges++;
                //ROS_DEBUG("Have %lu good edges for node %d", nodes[i].edges.size(), i);
            }
            k++;
        }
    }
    return nodes;
}

static inline bool clb_cmp(GraphNode * a, GraphNode * b)
{
    return (a->cost_lb < b->cost_lb); 
}


std::vector <GraphNode> solve_graph(std::vector<GraphNode> nodes)
{ 
    std::vector <GraphNode> spath; //To match up with Mike's interface for carrot controller
    //Now have set of nodes, edges
    //Create open set, closed set
    //Note that cost_lb lower bounds on cost are initialized to distance from that node to the 
    //end node already, so this does not have to be computed here.
    GraphNode start, end, cur;
    start = nodes[0];
    start.cost = 0;
    end = nodes[nodes.size() -1]; //From fn behavior above

    std::vector <GraphNode> os, cs;
    int ind = 0; 

    if (end.edges.size() == 0 || start.edges.size() == 0)
    {
       // ROS_DEBUG("Start or end point has no edges!");
        ROS_DEBUG("Start node has %lu edges", start.edges.size());
        ROS_DEBUG("End node has %lu edges", end.edges.size());
        
        ROS_DEBUG("Start at (%d, %d), map value: %f", start.x, start.y, map.valAt(start.x, start.y));
        ROS_DEBUG("End at (%d, %d), map value: %f", end.x, end.y, map.valAt(end.x, end.y));;
        
        return spath; //No way to reach end point
    }

    end = nodes[nodes.size() -1];
    cur = start; //Start node is current

    bool fail = false;
    ROS_DEBUG("Computing shortest path");
    // ROS_DEBUG("Starting node has %lu edges", start.edges.size());
    while(cur.id != end.id)
    {
        // ROS_DEBUG("A* for node id %d", cur.id);
        //Add current edges to open set if not already inside or if path is better
        for (int i = 0; i < cur.edges.size(); i++)
        {
            //ROS_DEBUG("Checking if id %d in sets", cur.edges[i]->id);

            double dx, dy, cost_lb, edge_len;
            dx = cur.x*1.0 - cur.edges[i]->x;
            dy = cur.y*1.0 - cur.edges[i]->y;
            edge_len = sqrt(dx*dx + dy*dy);

            dx = cur.edges[i]->x*1.0 - end.x;
            dy = cur.edges[i]->y*1.0 - end.y;
            cost_lb = sqrt(dx*dx + dy*dy) + cur.cost + edge_len;

            bool add_to_os = true;
            int to_remove = -1;
            for (int j = 0; j < os.size(); j++)
            {
                //Check if in open set, and add if cost_lb + cur.cost < cost
                if (os[j].id == cur.edges[i]->id) //Element already in open set
                {
                    // ROS_DEBUG("Element %d already in open set at index %d", os[j].id, j);
                    if(os[j].cost_lb < cost_lb ) //Found element has better cost
                    {   
                        //ROS_DEBUG("Old element is better");
                        add_to_os = false; //already a better path to this node in the OS
                    }
                    else //found element has worse cost
                    {
                        // ROS_DEBUG("Removing element %d from os", j); 
                        to_remove = j; //remove that version of it so it can be replaced below
                    }
                }
            }

            if (to_remove != -1)
            {
                os.erase(os.begin() + to_remove);
            }

            for(int j = 0; j < cs.size(); j++)
            {
                if(cs[j].id == cur.edges[i]->id)
                {
                    // ROS_DEBUG("Element %d already in closed set at index %d", cs[j].id, j);
                    add_to_os = false;
                }
            }

            if(cur.edges[i]->id == cur.id)
            {
                ROS_DEBUG("Got edge between %d and itself (%d)", cur.id, cur.edges[i]->id);
                add_to_os = false; //Got edge to itself
            }

            if(add_to_os)
            {
            int k = 0;
                //Find index to insert
                if (os.size() > 0)
                {
                    while( (cost_lb > os[k].cost_lb) && (k < os.size()) )
                    {
                        k++;
                    }
                }
                //Nodes at node.id gives back the whole node associated with that id
                //Not really safe, but works for limited implementation here
                nodes[cur.edges[i]->id].cost_lb = cost_lb;
                nodes[cur.edges[i]->id].cost = cur.cost + edge_len;
                nodes[cur.edges[i]->id].last_edge = &nodes[cur.id];

                // ROS_DEBUG("Insert %d into os at index %d of %lu", cur.edges[i]->id, k, os.size());
                // ROS_DEBUG("Back pointer to id %d", cur.id);
                if (k < os.size())
                    os.insert(os.begin() + k, nodes[cur.edges[i]->id]);
                else //Add elelment to end
                    os.push_back(nodes[cur.edges[i]->id]);
            }
        }

        cs.push_back(nodes[cur.id]);

        if (os.size() > 0)
        {
            cur = os[0];          //Take best value of open set
            os.erase(os.begin()); //Current node is first element, remove from open set
        }
        else
        {
            fail = true; //Nothing left in open set
            break;
        }

        if (cs.size() == nodes.size())
        {
            fail = true;
            break;
        }
    }

    if (fail)
    {
        return spath;
    }

    ROS_DEBUG("Done computing path");
    //ROS_DEBUG("Have id %d, with pointer to %d", cur.id, cur.last_edge->id);

    //Iterate through back pointers from the end node to the start node in the open set
    GraphNode * next = cur.last_edge;
    spath.push_back(cur);
    while(next != NULL)
    {
        //ROS_DEBUG("Adding node %d to shortest path", next->id);
        spath.push_back(*next);
        next = next->last_edge;
    }

    return spath;
}

static inline void map_to_global(int xin, int yin, double& xout, double& yout, OccGrid lmap)
{
    yout = xin * cell_size + lmap.orig_y;
    xout = yin * cell_size + lmap.orig_x;
}

static inline void global_to_map(double xin, double yin, int& xout, int& yout, OccGrid lmap)
{
    xout = yin/cell_size - lmap.orig_y/cell_size;
    yout = xin/cell_size - lmap.orig_x/cell_size;
}

//For use with nav_msgs::Point and other Ros data types
static inline void global_to_map(double xin, double yin, double& xout, double& yout, OccGrid lmap)
{
    xout = (int)(yin/cell_size - lmap.orig_y/cell_size); //Map indices must be integets
    yout = (int)(xin/cell_size - lmap.orig_x/cell_size);
}

void publish_path(std::vector<GraphNode> node_path)
{
    ros::Time latest = ros::Time(0);
    
    nav_msgs::Path pub_path;
    pub_path.header.seq++;
    pub_path.header.stamp = latest;
    pub_path.header.frame_id = "/map";
    
    geometry_msgs::PoseStamped pose;
    for(int i = 0; i < spath.size(); i++)
    {
        map_to_global(node_path[i].x, node_path[i].y, pose.pose.position.x, pose.pose.position.y, map);
        
        pose.header.seq++;
        pose.header.stamp = latest;
        pose.header.frame_id = "/map";
        
        pub_path.poses.insert(pub_path.poses.begin(), pose);
    }

    pose_pub.publish(pub_path);
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_x = msg.pose.position.x;
    goal_y = msg.pose.position.y;
    ROS_DEBUG("Got goal x, y = %0.2f, %0.2f", goal_x, goal_y);

    geometry_msgs::Point start_pt, end_pt;

    global_to_map(true_x, true_y, start_pt.x, start_pt.y, map);
    global_to_map(goal_x, goal_y, end_pt.x, end_pt.y, map);

    nodes = compute_edges(milestones, start_pt, end_pt, map);

    //nodes now populated with lists of edges, cost lower bounds, etc
    spath = solve_graph(nodes);
    publish_path(spath);
}

std::vector <geometry_msgs::Point> get_milestones(const OccGrid & map, const unsigned nM, 
                                                        const int pad_size)
{
    int s1x, s1y, s2x, s2y, midx, midy;
    std::vector <geometry_msgs::Point> pts;
    geometry_msgs::Point cur_pt;
    const int grid_ratio = 2; 

    ROS_DEBUG("Getting Milestones");

    int m_cur = 0;
    bool grid = true;   //Different logic for sampling
                        //In doing grid want fininte number of tries, in
                        //doing bridge want total number of successful samples
    while (m_cur < nM)
    {
        if ( m_cur < nM/grid_ratio )
        {
            //grid for first 1/2 of milestones
            int grid_dim = sqrt( (float)nM / grid_ratio ); //for square map
            midx = (m_cur % grid_dim) * (map.width - 2.0*(pad_size + 1))/(grid_dim - 1.0) + pad_size + 1;
            midy = (m_cur / grid_dim) * (map.width - 2.0*(pad_size + 1))/(grid_dim - 1.0) + pad_size + 1;

            //ROS_DEBUG("Grid sample at (%d, %d)", midx, midy);
            m_cur++;
        }
        else
        {
            grid = false;
            //ranodm bridge sampling for rest of milestones
            s1x = rand() * 1.0 * map.width / RAND_MAX;
            s1y = rand() * 1.0 * map.height / RAND_MAX;
            s2x = rand() * 1.0 * map.width / RAND_MAX;
            s2y = rand() * 1.0 * map.height / RAND_MAX;

            //ROS_DEBUG("s1 (%d, %d), s2 (%d, %d)", s1x, s1y, s2x, s2y);

            if (map.valAt(s1x, s1y) > occ_thresh && map.valAt(s2x, s2y) > occ_thresh)
            {
            //ROS_DEBUG("Both in obstacles");
            //Both vals are in obstacles
                midx = s1x + (s2x - s1x) / 2;
                midy = s1y + (s2y - s1y) / 2;
            }
            else
                midx = -1;
        }

        if (midx != -1)
        {
            //Check for obstacles +- pad size in x and y
            //Highly subobtimal compared to pre-padding the grid
            bool near_obst = false;
            for (int i = -1*pad_size; i < pad_size; i++)
            {
                for(int j = -1*pad_size; j < pad_size; j++)
                {
                    if ( map.valAt(midx + i, midy + j) > occ_thresh || map.valAt(midx + i, midy + j) == -1)
                    {
                        near_obst = true;
                        break;
                    }
                }
            }
            
            if (!near_obst) //Mid pt not in an obstacle
            {
                //ROS_DEBUG("Midpt (%d, %d) not in obstacle", midx, midy);
                cur_pt.x = midx;
                cur_pt.y = midy;
                pts.push_back(cur_pt);

                if (!grid)
                    m_cur++;
                //ROS_DEBUG("Found %d good milestones", m_cur);
            }            
        }
    }
    ROS_DEBUG("Found %d good milestones", m_cur);
    return pts;
}


//Callback function for the map
//This function is called when a new map is received
//I think this only happens when the subscriber first loads up, so can do sampling here
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    static const unsigned pad_size = 2; //3 cells in each direction to accommodate turtlebot width

    map.height = msg.info.height;
    map.width = msg.info.width;
    map.orig_x = msg.info.origin.position.x;
    map.orig_y = msg.info.origin.position.y;
    cell_size = msg.info.resolution;

    map.map_data = msg.data;
    test_occ_grid(map);

    milestones = get_milestones(map, nM, pad_size);
}


int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Set-up coordinate frame
    tf::TransformBroadcaster tf_br;
    tf::Transform transform;

    //Subscribe to the desired topics and assign callbacks
    #ifdef SIMULATION
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    #else
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    #endif

    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    //Setup topics to Publish from this node
    //ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    ros::Subscriber vel_sub = n.subscribe("/mobile_base/commands/velocity", 1, vel_callback);
    ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose_estimate", 1, true);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    pose_pub = n.advertise<nav_msgs::Path>("/path", 1, true);

    //callback for goal fn
    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, goal_callback);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    //Initialize points 
    //Graph solution, green
    visualization_msgs::Marker graph_soln;
    graph_soln.header.frame_id = "/map";
    graph_soln.id = 0;

    graph_soln.type = visualization_msgs::Marker::LINE_STRIP;
    graph_soln.scale.x = 0.15;
    graph_soln.scale.y = 0.15;

    graph_soln.color.g = 1.0f;
    graph_soln.color.a = 1.0;

    //Milestone points, blue
    visualization_msgs::Marker ms_points;
    ms_points.header.frame_id = "/map";
    ms_points.id = 1;

    ms_points.type = visualization_msgs::Marker::POINTS;
    ms_points.scale.x = 0.05;
    ms_points.scale.y = 0.05;

    ms_points.color.b = 1.0;
    ms_points.color.a = 0.8;  

    //True State marker, yellow
    visualization_msgs::Marker state_points;
    state_points.header.frame_id = "/map";
    state_points.id = 2;

    state_points.type = visualization_msgs::Marker::POINTS;
    state_points.scale.x = 0.1;
    state_points.scale.y = 0.1;

    state_points.color.r = 0.5;
    state_points.color.g = 0.5;
    state_points.color.a = 0.8;

    //True path points, yellow
    visualization_msgs::Marker true_path_line;
    true_path_line.header.frame_id = "/map";
    true_path_line.id = 3;

    true_path_line.type = visualization_msgs::Marker::LINE_STRIP;
    true_path_line.scale.x = 0.05;

    true_path_line.color.r = 0.5;
    true_path_line.color.g = 0.5;
    true_path_line.color.a = 0.8;

    //Graph edges, pink!
    visualization_msgs::Marker graph_edge;
    graph_edge.header.frame_id = "/map";
    graph_edge.id = 3;

    graph_edge.type = visualization_msgs::Marker::LINE_LIST;
    graph_edge.scale.x = 0.02;

    graph_edge.color.r = 1.0;
    graph_edge.color.g = 0;
    graph_edge.color.b = 1.0;
    graph_edge.color.a = 0.8;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        tf_br.sendTransform(tf::StampedTransform(transform, 
                                ros::Time::now(), "world", "my_frame"));
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        tf_br.sendTransform(tf::StampedTransform(transform, 
                                ros::Time::now(), "map", "my_frame"));

        //Points associated with true state and sensor data
        geometry_msgs::Point p;
        //ROS_DEBUG("Publishing robot state at (%f, %f)", true_x, true_y);
        p.x = true_x;
        p.y = true_y;
        p.z = 0;
        state_points.points.push_back(p);
        true_path_line.points.push_back(p);

    	// vel.linear.x = .1; // set linear speed
    	// vel.angular.z = .2; // set angular speed
        //ROS_DEBUG("Publishing %lu milestones", milestones.size());
        for (int i = 0; i < milestones.size(); i++)
        {
            //ROS_DEBUG("Publishing milestone at (%d, %d)", 
            map_to_global(milestones[i].x, milestones[i].y, p.x, p.y, map);
            ms_points.points.push_back(p);
        }

        for (int i = 0; i < nodes.size(); i++)
        {
            for (int j = 0; j < nodes[i].edges.size(); j++)
            {
                map_to_global(nodes[i].x, nodes[i].y, p.x, p.y, map);
                graph_edge.points.push_back(p);

                map_to_global(nodes[i].edges[j]->x, nodes[i].edges[j]->y, p.x, p.y, map);               
                graph_edge.points.push_back(p);

            }
        }

        for (int i = 0; i < spath.size(); i++)
        {
            map_to_global(spath[i].x, spath[i].y, p.x, p.y, map);
            graph_soln.points.push_back(p);
        }

    	//velocity_publisher.publish(vel); // Publish the command velocity
        if (vals_init)
        {
            marker_pub.publish(state_points);
            marker_pub.publish(ms_points);
            marker_pub.publish(true_path_line);
            marker_pub.publish(graph_edge);
            marker_pub.publish(graph_soln);
        }

        state_points.points.pop_back();
        graph_soln.points.clear();
        ms_points.points.clear();
        graph_edge.points.clear();
        graph_soln.points.clear();
    }

    ROS_DEBUG("Fell out of main loop!");

    return 0;
}
