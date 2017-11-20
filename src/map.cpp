/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include <lidar_eskf/map.h>

DistMap::DistMap(ros::NodeHandle &nh) {

    nh.param("map_file_name", _map_file_name, std::string("nsh_1109.bt"));
    nh.param("octree_resolution", _octree_resolution, 0.05);
    nh.param("max_obstacle_dist", _max_obstacle_dist, 0.5);

    _cloud_sub = nh.subscribe("/map_update", 1, &DistMap::cloud_callback, this);
    _octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    _map_ptr = boost::shared_ptr<octomap::OcTree> (new octomap::OcTree (_octree_resolution));

    read_mapfile();
    usleep(100);
}

void DistMap::read_mapfile() {
    std::fstream mapFile(_map_file_name.c_str(), std::ios_base::binary | std::ios_base::in);

    if (mapFile.is_open()) {
        ROS_INFO("DistMap: loading binary map.");

        _map_ptr->readBinary(_map_file_name);

        if(_map_ptr.get()) {
            if(!_map_ptr || _map_ptr->size() <= 1) {
                ROS_ERROR("Load distance file \"%s\" failed.", _map_file_name.c_str());
                exit(-1);
            }
        }

        init_dist_map();
        mapFile.close();
    }
    else {
        ROS_ERROR("OctoMap file \"%s\" is not open.", _map_file_name.c_str());
        exit(-1);
    }
}

boost::shared_ptr<octomap::OcTree> DistMap::get_map() const{
    return _map_ptr;
}

 boost::shared_ptr<DynamicEDTOctomap> DistMap::get_dist_map() const{
    return _dist_map_ptr;
}

void DistMap::init_dist_map() {
    double x, y, z;
    _map_ptr->getMetricMin ( x, y, z );
    octomap::point3d min ( x, y, z );
    min(0) -= _max_obstacle_dist;
    min(1) -= _max_obstacle_dist;
    min(2) -= _max_obstacle_dist;
    _map_ptr->getMetricMax ( x, y, z );
    octomap::point3d max ( x, y, z );
    max(0) += _max_obstacle_dist;
    max(1) += _max_obstacle_dist;
    max(2) += _max_obstacle_dist;

    _dist_map_ptr = boost::shared_ptr<DynamicEDTOctomap> (
                      new DynamicEDTOctomap ( float ( _max_obstacle_dist ), & ( *_map_ptr ), min, max, false ) );
    _dist_map_ptr->update();

    ROS_INFO("DistMap: Initialization done.");
    ROS_INFO("DistMap: Distance map range:");
    ROS_INFO("         min = [%0.3f %0.3f %0.3f]", min(0), min(1), min(2));
    ROS_INFO("         max = [%0.3f %0.3f %0.3f]", max(0), max(1), max(2));


}

double DistMap::ray_casting(octomap::point3d endPt, octomap::point3d originPt, octomap::point3d &rayEndPt) {
    double dist = 0;
    octomap::point3d direction;
    direction = endPt - originPt;

    if(_map_ptr->castRay(originPt, direction, rayEndPt, true, 15.0)) {
        dist = (rayEndPt - endPt).norm();
    }
    else {
        dist = -1.0;
        ROS_WARN("DistMap: ray_cast fails");
    }
    return dist;
}

double DistMap::get_dist(octomap::point3d p) {
    return _dist_map_ptr->getDistance(p);
}
char DistMap::get_gridmask(octomap::point3d p) {
    octomap::OcTreeKey key = _map_ptr->coordToKey(p);
    octomap::OcTreeNode* node = _map_ptr->search(key);
    if(!node) {
        return 2;
    } else if(_map_ptr->isNodeOccupied(node)) {
        return 1;
    } else {
        return 0;
    }
}

void DistMap::cloud_callback(const sensor_msgs::PointCloud2 &msg) {
    octomap::Pointcloud cloud;
    octomap::pointCloud2ToOctomap(msg, cloud);

    tf::StampedTransform transform;
    try{
      _listener.lookupTransform("/world", "/laser_m",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    tf::Vector3 translation;
    translation = transform.getOrigin();
    double x, y, z;
    x = translation.x(); y = translation.y(); z = translation.z();
    tf::Matrix3x3 rotation;
    rotation.setRotation(transform.getRotation());
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    octomap::point3d sensor_origin(0.0,0.0,0.0);
    octomap::pose6d  frame_pose(x, y, z, roll, pitch, yaw);
    _map_ptr->insertPointCloud(cloud, sensor_origin, frame_pose);
    _map_ptr->updateInnerOccupancy();
    _dist_map_ptr->update();

    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::binaryMapToMsg(*_map_ptr, octomap_msg);
    octomap_msg.header.frame_id = "world";
    octomap_msg.header.stamp = msg.header.stamp;
    octomap_msg.id = 1;
    octomap_msg.binary = 1;
    octomap_msg.resolution = _map_ptr->getResolution();
    _octomap_pub.publish(octomap_msg);
    ROS_INFO("DistMap: cloud_callback(): update distance map. map leaf node size %lu", _map_ptr->getNumLeafNodes());

}
