/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef ESKF_H
#define ESKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iostream>
#include <time.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>
#include <numeric>

class ESKF {
public:
    ESKF(ros::NodeHandle &nh);
    ~ESKF();

    void imu_callback(const sensor_msgs::Imu &msg);
    void update_time(const sensor_msgs::Imu &msg);
    void update_imu(const sensor_msgs::Imu &msg);
    void propagate_error();
    void propagate_covariance();
    void propagate_state();
    void get_mean_pose(Eigen::Matrix<double, 6, 1> &mean_pose);
    void get_mean_pose(Eigen::Matrix<double, 7, 1> &mean_pose);
    void get_cov_pose(Eigen::Matrix<double, 6, 6> &cov_pose);
    void publish_odom();
    void publish_bias();

    void update_meas_mean(Eigen::Matrix<double, 6, 1> &mean_meas);
    void update_meas_cov(Eigen::Matrix<double, 6, 6> &cov_meas);
    void update_meas_flag();
    void update_error();
    void update_state();
    void reset_error();
    void output_log();

private:

    // nominal states
    Eigen::Vector3d _velocity;
    Eigen::Matrix3d _rotation;
    Eigen::Quaterniond _quaternion;
    Eigen::Vector3d _position;
    Eigen::Vector3d _bias_acc;
    Eigen::Vector3d _bias_gyr;


    // error states
    Eigen::Vector3d   _d_velocity;
    Eigen::Vector3d   _d_theta;
    Eigen::Matrix3d   _d_rotation;
    Eigen::Vector3d   _d_position;
    Eigen::Vector3d   _d_bias_acc;
    Eigen::Vector3d   _d_bias_gyr;

    // imu measurements
    Eigen::Vector3d    _imu_acceleration;
    Eigen::Vector3d    _imu_angular_velocity;
    Eigen::Quaterniond _imu_orientation;

    // jacobian matrices
    Eigen::Matrix<double, 15, 15> _Fx;
    Eigen::Matrix<double, 15, 12> _Fn;

    // covariance matrix
    Eigen::Matrix<double, 15, 15> _Sigma;
    Eigen::Matrix<double, 12, 12> _Q;

    // gravity
    double _g;
    Eigen::Vector3d _gravity;

    // time relatives
    ros::Time _imu_time; 
    double _dt, _imu_freq;
    bool _init_time;

    // noise params
    double _sigma_acc, _sigma_gyr;
    double _sigma_bias_acc, _sigma_bias_gyr;

    // initialization params
    double _init_bias_acc_x, _init_bias_acc_y, _init_bias_acc_z;
    double _init_roll, _init_pitch, _init_yaw;
    // subscriber and publisher
    ros::Subscriber _imu_sub;
    ros::Publisher  _odom_pub, _bias_pub;

    // odometry measurements
    Eigen::Vector3d    _m_theta;
    Eigen::Vector3d    _m_position;
    bool _got_measurements;

    Eigen::Matrix<double, 6, 6> _m_pose_sigma;

    // a queue to smooth imu accleration measurements
    std::vector<geometry_msgs::Vector3> _acc_queue;
    int _acc_queue_size;
    int _acc_queue_count;

    // log of odom
    std::vector<nav_msgs::Odometry> _odom_vec;
    
    // frames
    std::string _imu_frame, _robot_frame;
   
    // imu related
    bool _imu_enabled, _imu_has_quat, _imu_transform;
    tf::TransformListener _tf_listener;

    // smoother
    bool _smooth_enabled;
    int _smooth_buf_size;
    int _smooth_buf_cnt;
    std::string _smooth_type;
    std::vector<double> _x_buf;
    std::vector<double> _y_buf;
    std::vector<double> _z_buf;
    std::vector<double> _vx_buf;
    std::vector<double> _vy_buf;
    std::vector<double> _vz_buf;

};

Eigen::Matrix3d skew(Eigen::Vector3d w);
Eigen::Matrix3d angle_axis_to_rotation_matrix(Eigen::Vector3d w);
Eigen::Matrix3d euler_angle_to_rotation_matrix(Eigen::Vector3d w);

#endif // IMUODOM_H
