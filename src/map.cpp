#include <lidar_eskf/map.h>

DistMap::DistMap(ros::NodeHandle &nh) {

    nh.param("map_file_name", _map_file_name, std::string("/home/slz/inspection_ws/src/lidar_eskf/map/bridge.bt"));
    nh.param("octree_resolution", _octree_resolution, 0.05);
    nh.param("max_obstacle_dist", _max_obstacle_dist, 10.0);
    std::cout << _max_obstacle_dist << std::endl;
    read_mapfile();
    usleep(100);
}

void DistMap::read_mapfile() {
    // std::fstream mapFile(_map_file_name.c_str(), std::ios_base::binary | std::ios_base::in);
    std::fstream mapFile("/home/slz/inspection_ws/src/lidar_eskf/map/bridge.bt", std::ios_base::binary | std::ios_base::in);
    // std::cout << _map_file_name.c_str() << std::endl;
    if (mapFile.is_open()) {
        ROS_INFO("DistMap: loading binary map.");
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

    ROS_INFO("DistMap: min = %0.3f %0.3f %0.3f; max = %0.3f %0.3f %0.3f",
             min(0), min(1), min(2), max(0), max(1), max(2));
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

double DistMap::get_dist(octomap::point3d p) {
    return _dist_map_ptr->getDist(p);
}
char DistMap::get_gridmask(octomap::point3d p) {
    return _dist_map_ptr->getGridMask(p);
}
void DistMap::get_closest_obstacle(octomap::point3d p, float &distance, octomap::point3d& closestObstacle){
    _dist_map_ptr->getDistanceAndClosestObstacle(p, distance, closestObstacle);
}
