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

/*class MapModel: public DynamicEDTOctomap
{
public:
    MapModel(float max_dist,
             octomap::OcTree* octree,
             octomap::point3d bbx_min,
             octomap::point3d bbx_max,
             bool treat_unknown_occupied):
    _max_dist(max_dist),
    _octree(octree),
    _bbx_min(bbx_min),
    _bbx_max(bbx_max),
    _treat_unknown_occupied(treat_unknown_occupied),
    DynamicEDTOctomap(max_dist, octree, bbx_min, bbx_max, treat_unknown_occupied) {}

    virtual ~MapModel() {
        _octree = NULL;
    }
    virtual void update(bool updateRealDist=true, bool updateGridMask=true) {
        DynamicEDTOctomap::update(updateRealDist);
        ROS_INFO("Update DistMap.");
        update_gridmask();
    }
    void update_gridmask() {

        _size_x = boundingBoxMaxKey[0] - boundingBoxMinKey[0] + 1;
        _size_y = boundingBoxMaxKey[1] - boundingBoxMinKey[1] + 1;
        _size_z = boundingBoxMaxKey[2] - boundingBoxMinKey[2] + 1;

        _gridMask.resize(_size_x);
        for ( int x=0; x<_size_x; x++ )
        {
           _gridMask[x].resize(_size_y);
           for ( int y=0; y<_size_y; y++ )
              _gridMask[x][y].resize(_size_z);
        }

        octomap::OcTreeKey key;
        for ( int dx=0; dx<_size_x; dx++ )
        {
           key[0] = boundingBoxMinKey[0] + dx;
           for ( int dy=0; dy<_size_y; dy++ )
           {
              key[1] = boundingBoxMinKey[1] + dy;
              for ( int dz=0; dz<_size_z; dz++ )
              {
                 key[2] = boundingBoxMinKey[2] + dz;

                 octomap::OcTreeNode* node = _octree->search ( key );
                 if ( !node )
                 {
                     // unknown grid
                     _gridMask[dx][dy][dz] = 2;
                 }
                 else if(_octree->isNodeOccupied ( node ))
                 {
                     // occupied grid
                     _gridMask[dx][dy][dz] = 1;
                 }
                 else
                 {
                     // free grid
                     _gridMask[dx][dy][dz] = 0;
                 }
              }
           }
        }
    }

    char getGridMask(octomap::point3d& p) const {
        int x,y,z;
        worldToMap ( p, x, y, z );
        if ( x>=0 && x<_size_x && y>=0 && y<_size_y && z>=0 && z<_size_z )
        {
           return _gridMask[x][y][z];
        }
        else
        {
           return 2;
        }
    }

protected:
    float _max_dist;
    octomap::OcTree* _octree;
    octomap::point3d _bbx_min;
    octomap::point3d _bbx_max;
    bool _treat_unknown_occupied;
    int _size_x;
    int _size_y;
    int _size_z;

    std::vector<std::vector<std::vector<char> > > _gridMask;
};*/


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
//    boost::shared_ptr<MapModel> get_dist_map() const;
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
//    boost::shared_ptr<octomap::OcTree> _map_dup_ptr;

    // Distance map pointer
//    boost::shared_ptr<MapModel> _dist_map_ptr;
    boost::shared_ptr<DynamicEDTOctomap> _dist_map_ptr;

    // Octomap Subscriber
    ros::Subscriber _cloud_sub;

    // TF listener
    tf::TransformListener _listener;

    // Map Publisher
    ros::Publisher _octomap_pub;

};

#endif // MAP_H
