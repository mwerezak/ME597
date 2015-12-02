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
#include <nav_msgs/Odometry.h>
#include <random>
#include <tf/transform_broadcaster.h>

#define SIMULATION //comment out when running on turtlebot

const int N_particles = 50;
const double dt = 0.05; //20 Hz
const int n_states = 3;  //number of states

ros::Publisher dithered_ips_pub;

struct Particle
{
    double x;
    double y;
    double yaw;
    double wx;
    double wy;
    double wyaw;
};

Particle particles[N_particles];

bool vals_init = false; //first IPS/state measurement used to init odom and particle filter
bool new_ips = false;

//set up Gaussians
std::default_random_engine generator;

//Need to compute Q for dithering
const double Q = 0.01;              //No dithering, noise for x and y, assume decoupled
const double Qt = 0.01*M_PI / 180;   //noise for theta, assume decoupled
const double R = 0.2;                //disturbance variance in x
const double Rt = 0.2 * M_PI/ 180;   //theta disturbance variance

double ips_x, ips_y, ips_yaw;
double true_x, true_y, true_yaw;

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

void init()
{
    // get starting position for the odom
    // has the benefit of not needing transforms for Aodom
    odom_x = ips_x;
    odom_y = ips_y;
    odom_yaw = ips_yaw;


    ROS_DEBUG("init with global pts\n: X %f Y: %f Yaw: %f", 
                    odom_x, odom_y, odom_yaw);
    
    std::normal_distribution<double> particle_scatter(0, 5*Q); //5 sigma about mean
    std::normal_distribution<double> particle_angle_scatter(0, 5*Qt);    

    //initialize particle filter
    for (int i = 0; i < N_particles; i++)
    {
        particles[i].x = ips_x + particle_scatter(generator);
        particles[i].y = ips_y + particle_scatter(generator);
        particles[i].yaw = ips_yaw + particle_angle_scatter(generator); 
    }

    vals_init = true;
}

void publish_ips()
{
    static geometry_msgs::PoseStamped ips_dith;
    ips_dith.header.stamp = ros::Time::now();
    ips_dith.pose.position.x = ips_x;
    ips_dith.pose.position.y = ips_y;
    dithered_ips_pub.publish(ips_dith);
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

//    ROS_DEBUG("x dither: %f\n y dither %f\n", x_dith, y_dith);

    ips_x = true_x;
    ips_y = true_y;
    ips_yaw = true_yaw; //No dither required
    
    if (!vals_init) init();
    publish_ips();

    // ROS_DEBUG("ips callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
    new_ips = true;
}

#else
//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    const double scaling = 2.17;

    true_x = scaling * msg.pose.pose.position.x; // Robot X psotition
    true_y = scaling * msg.pose.pose.position.y; // Robot Y psotition
    true_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

//    ROS_DEBUG("x dither: %f\n y dither %f\n", x_dith, y_dith);

    ips_x = true_x;
    ips_y = true_y;     //reusing publisher from lab 2
    ips_yaw = true_yaw; //No dithering required for this lab, 
    
    if (!vals_init) init();
    publish_ips();
    //ROS_DEBUG("ips callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
    new_ips = true;
}
#endif

void odom_callback(const nav_msgs::Odometry& msg)
{
    static nav_msgs::Odometry last_msg;

    if(!vals_init) return;

    odom_vx = msg.twist.twist.linear.x;
    odom_vy = msg.twist.twist.linear.y;
    odom_w = msg.twist.twist.angular.z;

    #ifdef SIMULATION
    //zero order time integration
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

    last_msg = msg;
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

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_x = msg.pose.position.x;
    goal_y = msg.pose.position.y;
    ROS_DEBUG("Got goal x, y = %0.2f, %0.2f", goal_x, goal_y);
}


int my_find(double * a, double val, int size)
{   
    //find first n > val in sorted array
    int i = 0;
    while(val > a[i])
    {
        i++;
        if (i >= size)
        {
            i = -1;
            break;
        }
    }
    return i;
}

int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                                        ros::console::levels::Debug) ) {
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

    //Setup topics to Publish from this node
    //ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    ros::Subscriber vel_sub = n.subscribe("/mobile_base/commands/velocity", 1, vel_callback);
    ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose_estimate", 1, true);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    dithered_ips_pub = n.advertise<geometry_msgs::PoseStamped>("/dithered_ips", 1, true);

    //callback for goal fn
    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, goal_callback);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    //Initialize points 
    //Dithered position measurement, green
    visualization_msgs::Marker ips_points;
    ips_points.header.frame_id = "/map";
    ips_points.id = 0;

    ips_points.type = visualization_msgs::Marker::POINTS;
    ips_points.scale.x = 0.1;
    ips_points.scale.y = 0.1;

    ips_points.color.g = 1.0f;
    ips_points.color.a = 1.0;

    //Odometry measurement, blue
    visualization_msgs::Marker odom_points;
    odom_points.header.frame_id = "/map";
    odom_points.id = 1;

    odom_points.type = visualization_msgs::Marker::POINTS;
    odom_points.scale.x = 0.1;
    odom_points.scale.y = 0.1;

    odom_points.color.b = 1.0;
    odom_points.color.a = 0.8;

    //Belief visualization, red
    visualization_msgs::Marker belief_points;
    belief_points.header.frame_id = "/map";
    belief_points.id = 2;

    belief_points.type = visualization_msgs::Marker::POINTS;
    belief_points.scale.x = 0.05;
    belief_points.scale.y = 0.05;

    belief_points.color.r = 1.0;
    belief_points.color.a = 0.8;  

    //True belief path , yellow
    visualization_msgs::Marker belief_line;
    belief_line.header.frame_id = "/map";
    belief_line.id = 3;

    belief_line.type = visualization_msgs::Marker::LINE_STRIP;
    belief_line.scale.x = 0.02;

    belief_line.color.r = 1.0;
    belief_line.color.a = 0.8;

    //State marker, yellow
    visualization_msgs::Marker state_points;
    state_points.header.frame_id = "/map";
    state_points.id = 4;

    state_points.type = visualization_msgs::Marker::POINTS;
    state_points.scale.x = 0.1;
    state_points.scale.y = 0.1;

    state_points.color.r = 0.5;
    state_points.color.g = 0.5;
    state_points.color.a = 0.8;


    //True path points, yellow
    visualization_msgs::Marker true_path_line;
    true_path_line.header.frame_id = "/map";
    true_path_line.id = 5;

    true_path_line.type = visualization_msgs::Marker::LINE_STRIP;
    true_path_line.scale.x = 0.02;

    true_path_line.color.r = 0.5;
    true_path_line.color.g = 0.5;
    true_path_line.color.a = 0.8;

    //Particle filter stuff
    std::normal_distribution<double> xy_disturbances(0, R);
    std::normal_distribution<double> theta_disturbances(0, Rt);

    double pdfs[n_states];
    double Wx[N_particles];
    double Wy[N_particles];
    double Wyaw[N_particles];
    double sigmas[n_states] = { Q, Q, Qt}; //saves some time below
    double seed;


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
        p.x = ips_x;
        p.y = ips_y;
        p.z = 0;
        ips_points.points.push_back(p);

        p.x = odom_x;
        p.y = odom_y;
        p.z = 0;
        odom_points.points.push_back(p);

        p.x = true_x;
        p.y = true_y;
        p.z = 0;
        state_points.points.push_back(p);
        true_path_line.points.push_back(p);

        geometry_msgs::PoseStamped pose_est;

    	// vel.linear.x = .1; // set linear speed
    	// vel.angular.z = .2; // set angular speed

        // //Particle filter
        double x_tot = 0; //for plotting mean
        double y_tot = 0;
        double yaw_tot = 0;
        for (int i = 0; i < N_particles; i++)
        {

            particles[i].x = particles[i].x + cos(particles[i].yaw) * odom_vx * dt  + xy_disturbances(generator);
            particles[i].y = particles[i].y + sin(particles[i].yaw) * odom_vx * dt  + xy_disturbances(generator);
            #ifdef SIMULATION
            particles[i].yaw = particles[i].yaw + odom_w * dt + theta_disturbances(generator);
            #else
            particles[i].yaw = particles[i].yaw - odom_w * dt + theta_disturbances(generator);
            #endif

            
            if (particles[i].yaw > M_PI)
                particles[i].yaw -= 2*M_PI;

            x_tot += particles[i].x;
            y_tot += particles[i].y;
            yaw_tot += particles[i].yaw;
        }

        p.x = x_tot / N_particles;
        p.y = y_tot / N_particles;

        pose_est.pose.position.x = p.x;
        pose_est.pose.position.y = p.y;
        pose_est.header.stamp = ros::Time::now();

        belief_line.points.push_back(p);

        if (new_ips) //Only re-sample when new pos data available
        {
            //ROS_DEBUG("new ips data.. Re-sampling");
            //Compute weights pushed to own loop so it's avoided if no new data available
            for (int i = 0; i < N_particles; i++)
            {
                particles[i].wx = normal_pdf(ips_x, particles[i].x, Q);
                particles[i].wy = normal_pdf(ips_y, particles[i].y, Q);
                particles[i].wyaw = normal_pdf(ips_yaw, particles[i].yaw, Qt);

                // if (particles[i].wx < 0.00000001)
                // {
                //     particles[i].wx = 0.00000001;
                // }                

                // if (particles[i].wy < 0.00000001)
                // {
                //     particles[i].wy = 0.00000001;
                // }                

                // if (particles[i].wyaw < 0.00000001)
                // {
                //     particles[i].wyaw = 0.00000001;
                // }

                if (i > 0)
                {
                    //compute cumulative sum fo weights
                    Wx[i] = Wx[i-1] + particles[i].wx;
                    Wy[i] = Wy[i-1] + particles[i].wy;
                    Wyaw[i] = Wyaw[i-1] + particles[i].wyaw;

                }
                else
                {   
                    Wx[i] = particles[i].wx;
                    Wy[i] = particles[i].wy;
                    Wyaw[i] = particles[i].wyaw;
                }

                // ROS_DEBUG("weight %d = %f", i, particles[i].weight);
                // ROS_DEBUG("W[%d] = %f", i, W[i]);
            }

            //Resample
            for (int i = 0; i < N_particles; i++)
            {
                //Re-sample X
                seed = (double)rand() / RAND_MAX * Wx[N_particles - 1];
                // ROS_DEBUG("seed = %f", seed);
                int index = my_find(Wx, seed, N_particles);
                // ROS_DEBUG("value %f found at index %d, \n\t(actual: %f)", 
                //     seed, index, W[index]);
                particles[i].x = particles[index].x; //resample!!

                //Re-sample y
                seed = (double)rand() / RAND_MAX * Wy[N_particles - 1];
                // ROS_DEBUG("seed = %f", seed);
                index = my_find(Wy, seed, N_particles);
                // ROS_DEBUG("value %f found at index %d, \n\t(actual: %f)", 
                //     seed, index, W[index]);
                particles[i].y = particles[index].y; //resample y!!

                //Re-sample yaw
                seed = (double)rand() / RAND_MAX * Wyaw[N_particles - 1];
                // ROS_DEBUG("seed = %f", seed);
                index = my_find(Wyaw, seed, N_particles);
                // ROS_DEBUG("value %f found at index %d, \n\t(actual: %f)", 
                //     seed, index, W[index]);

                particles[i].yaw = particles[index].yaw; //resample!!
            }    
            new_ips = false;           
        }

        for (int i = 0; i < N_particles; i++)
        {
            p.x = particles[i].x;
            p.y = particles[i].y;

            belief_points.points.push_back(p);
        }

    	//velocity_publisher.publish(vel); // Publish the command velocity
        if (vals_init)
        {
            marker_pub.publish(ips_points);
            marker_pub.publish(odom_points);
            marker_pub.publish(state_points);
            marker_pub.publish(belief_points);
            marker_pub.publish(true_path_line);
            marker_pub.publish(belief_line);
            pose_publisher.publish(pose_est);
        }

        odom_points.points.pop_back();
        ips_points.points.pop_back();
        state_points.points.pop_back();

        for (int i = 0; i < N_particles; i++)
        {
            belief_points.points.pop_back();
        }
    }

    ROS_DEBUG("Fell out of main loop!");

    return 0;
}
