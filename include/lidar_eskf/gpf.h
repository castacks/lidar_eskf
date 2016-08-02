#ifndef GPF_H
#define GPF_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

#include "lidar_eskf/particles.h"

class GPF {
public:
    GPF(ros::NodeHandle &nh, DistMap &map);
    ~GPF() {}

    void odom_callback(const nav_msgs::Odometry &msg);
    void cloud_callback(const sensor_msgs::PointCloud2 &msg);

    void downsample();
    void recover_pseudo_meas();
    void check_posdef(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &R);

private:

    Eigen::Matrix<double, 6, 1> _mean_prior;
    Eigen::Matrix<double, 6, 6> _cov_prior;
    Eigen::Matrix<double, 6, 1> _mean_posterior;
    Eigen::Matrix<double, 6, 6> _cov_posterior;
    Eigen::Matrix<double, 6, 1> _mean_meas;
    Eigen::Matrix<double, 6, 6> _cov_meas;

    tf::Vector3 _position_prior;
    tf::Vector3 _theta_prior;
    tf::Matrix3x3 _rotation_prior;
    tf::Quaternion _quaternion_prior;

    ros::Subscriber _cloud_sub;
    ros::Subscriber _odom_sub;
    ros::Publisher  _meas_pub;

    ros::Time _laser_time;

    boost::shared_ptr<DistMap> _map_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;

};
#endif // GPF_H
