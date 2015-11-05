#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "occupancy_grid.h"

const std::string IPS_FRAME = "ips"; //Frame ID for the robot position provided by IPS

//Updates the given occupancy map with the new information provided by the given laser scan.
void MappingUpdate(OccupancyGrid& occ_map, const sensor_msgs::LaserScan& scan_data);

//Updates the map given a beam that hit a feature at end
void MapUpdateBeamHit(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& hit_feature);

//Updates the map given a beam that hit nothing
void MapUpdateBeamMiss(OccupancyGrid& occ_map, const tf::Vector3& beam_start, const tf::Vector3& beam_end);

#endif