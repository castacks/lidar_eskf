/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef GPF_H
#define GPF_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iostream>
#include <deque>

#include "lidar_eskf/eskf.h"
#include "lidar_eskf/particles.h"

class GPF {
public:
    GPF(ros::NodeHandle &nh, boost::shared_ptr<DistMap> map_ptr);
    ~GPF();

    void pozyx_callback(const geometry_msgs::PoseWithCovariance &msg);
    void cloud_callback(const sensor_msgs::PointCloud2 &msg);
    void scan_callback(const sensor_msgs::LaserScan &msg);
    void downsample();
    void recover_meas();
    void check_posdef(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &R);
    void publish_cloud();
    void publish_posterior();
    void publish_meas();
    void publish_pset();
    void publish_path();
    void publish_tf();
    void publish_pose();
    std::vector< std::vector<double> > compute_color(Particles pSet);

private:

    Eigen::Matrix<double, 7, 1> _mean_prior; // nominal
    Eigen::Matrix<double, 6, 6> _cov_prior;  // error
    Eigen::Matrix<double, 6, 1> _mean_sample;
    Eigen::Matrix<double, 6, 6> _cov_sample;
    Eigen::Matrix<double, 6, 1> _mean_posterior;
    Eigen::Matrix<double, 6, 6> _cov_posterior;
    Eigen::Matrix<double, 6, 1> _mean_meas;
    Eigen::Matrix<double, 6, 6> _cov_meas;

    ros::Subscriber _pozyx_sub;
    ros::Subscriber _cloud_sub;
    ros::Subscriber _scan_sub;
    ros::Publisher  _cloud_pub;
    ros::Publisher  _meas_pub;
    ros::Publisher  _pset_pub;
    ros::Publisher  _post_pub;
    ros::Publisher  _path_pub;
    ros::Publisher  _pose_pub;

    laser_geometry::LaserProjection _projector;
    tf::TransformListener _listener;
    std::string _laser_type;
    ros::Time _laser_time;
    std::string _robot_frame;

    boost::shared_ptr<DistMap>          _map_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
    boost::shared_ptr<ESKF>             _eskf_ptr;
    boost::shared_ptr<Particles>        _particles_ptr;

    double _cloud_resol;
    double _ray_sigma;
    int    _set_size;
    double _cloud_range;

    nav_msgs::Path _path;
    std::deque<geometry_msgs::PoseStamped> _pose_deque;
    tf::TransformBroadcaster _tf_br;
};
#endif // GPF_H
