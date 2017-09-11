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

    /**
       * \brief Constructor
       * @param nh the ros node handle
    */
    DistMap(ros::NodeHandle &nh);

    /**
       * \brief Destructor
    */
    ~DistMap(){}

    /**
       * \brief Binary file reader
    */
    void read_mapfile();

    /**
       * \brief Return octomap pointer
    */
    boost::shared_ptr<octomap::OcTree> get_map() const;

    /**
       * \brief Return distance map pointer
    */
    boost::shared_ptr<DynamicEDTOctomap> get_dist_map() const;
    /**
       * \brief Convert octomap to distance map
    */
    void init_dist_map();

    /**
       * \brief Ray-casting method
       * This implementation does not work well
       * @param endPt the end point coordinates of measurement
       *        originPt the lidar center coordinates
       *        rayEndPt the end point of measurement ray in the map
       * @return Distance in meters
    */
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
