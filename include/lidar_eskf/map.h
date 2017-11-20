/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/transform_listener.h>

class DistMap
{
public:

    DistMap(ros::NodeHandle &nh);
    ~DistMap(){}

    void read_mapfile();
    boost::shared_ptr<octomap::OcTree> get_map() const;
    boost::shared_ptr<DynamicEDTOctomap> get_dist_map() const;
    void init_dist_map();
    double ray_casting(octomap::point3d endPt, octomap::point3d originPt, octomap::point3d &rayEndPt);
    double get_dist(octomap::point3d p);
    char get_gridmask(octomap::point3d p);
    void cloud_callback(const sensor_msgs::PointCloud2 &msg);
    
private:

    // File name of the binary octomap (*.bt)
    std::string _map_file_name;
    double _octree_resolution;
    double _max_obstacle_dist;

    // Octomap pointer
    boost::shared_ptr<octomap::OcTree> _map_ptr;

    // Distance map pointer
    boost::shared_ptr<DynamicEDTOctomap> _dist_map_ptr;

    // Octomap Subscriber
    ros::Subscriber _cloud_sub;

    // TF listener
    tf::TransformListener _listener;

    // Map Publisher
    ros::Publisher _octomap_pub;

};

#endif // MAP_H
