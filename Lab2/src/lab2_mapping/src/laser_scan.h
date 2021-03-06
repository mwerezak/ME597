#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "occupancy_grid.h"

const std::string IPS_FRAME = "ips"; //Frame ID for the robot position provided by IPS

//Updates the given occupancy map with the new information provided by the given laser scan.
void UpdateMapFromScan(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data);

bool _checkValidBeam(int beam_idx, const sensor_msgs::LaserScan& scan_data);
tf::Vector3 _getBeamHitPos(double beam_range, double angle);

//Updates the map given a beam that hit a feature at end
void _mapUpdateBeamHit(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& hit_feature);

#endif