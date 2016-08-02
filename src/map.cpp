#include <lidar_eskf/map.h>

DistMap::DistMap(ros::NodeHandle &nh) {

    nh.param("map_file_dist", _map_file_name, std::string("nsh_1109.bt"));
    nh.param("octree_resolution", _octree_resolution, 0.05);
    nh.param("max_obstacle_dist", _max_obstacle_dist, 0.5);
    read_mapfile();
    usleep(100);
}

void DistMap::read_mapfile() {
    std::fstream mapFile(_map_file_name.c_str(), std::ios_base::binary | std::ios_base::in);

    if (mapFile.is_open()) {
        _map_ptr = boost::shared_ptr<octomap::OcTree> (new octomap::OcTree (_octree_resolution));
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

boost::shared_ptr<MapModel> DistMap::get_dist_map() const {
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

    _dist_map_ptr = boost::shared_ptr<MapModel> (
                      new MapModel ( float ( _max_obstacle_dist ), & ( *_map_ptr ), min, max, false ) );

    _dist_map_ptr->update();
    ROS_INFO("DistMap: initialization done!");
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
