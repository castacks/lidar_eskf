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
#include <visualization_msgs/MarkerArray.h>

#include "lidar_eskf/eskf.h"
#include "lidar_eskf/particles.h"

class GPF {
public:
    GPF(ros::NodeHandle &nh, boost::shared_ptr<DistMap> map_ptr);
    ~GPF() {}

    void cloud_callback(const sensor_msgs::PointCloud2 &msg);

    void downsample();
    void recover_meas();
    void check_posdef(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &R);
    void publish_cloud();
    void publish_posterior();
    void publish_meas();
    void publish_pset();
    void publish_path();
    std::vector< std::vector<double> > compute_color(Particles pSet);

private:

    Eigen::Matrix<double, 6, 1> _mean_prior;
    Eigen::Matrix<double, 6, 6> _cov_prior;
    Eigen::Matrix<double, 6, 1> _mean_sample;
    Eigen::Matrix<double, 6, 6> _cov_sample;
    Eigen::Matrix<double, 6, 1> _mean_posterior;
    Eigen::Matrix<double, 6, 6> _cov_posterior;
    Eigen::Matrix<double, 6, 1> _mean_meas;
    Eigen::Matrix<double, 6, 6> _cov_meas;

    ros::Subscriber _cloud_sub;
    ros::Publisher  _cloud_pub;
    ros::Publisher  _meas_pub;
    ros::Publisher  _pset_pub;
    ros::Publisher  _post_pub;
    ros::Publisher  _path_pub;

    ros::Time _laser_time;

    boost::shared_ptr<DistMap>          _map_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
    boost::shared_ptr<ESKF>             _eskf_ptr;
    boost::shared_ptr<Particles>        _particles_ptr;

    double _cloud_resol;
    double _ray_sigma;
    int    _set_size;
    double _cloud_range;

    double _imu_to_laser_roll;
    double _imu_to_laser_pitch;
    double _imu_to_laser_yaw;
    tf::Matrix3x3 _imu_to_laser_rotation;
    Eigen::Matrix<double, 4, 4> _imu_to_laser_transform;

    nav_msgs::Path _path;

};
#endif // GPF_H
