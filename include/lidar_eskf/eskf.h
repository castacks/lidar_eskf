#ifndef ESKF_H
#define ESKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <queue>
#include <boost/circular_buffer.hpp>

class ESKF {
public:
    ESKF(ros::NodeHandle &nh);
    ~ESKF() {}

    void imu_callback(const sensor_msgs::Imu &msg);
    void update_time(const sensor_msgs::Imu &msg);
    void update_imu(const sensor_msgs::Imu &msg);
    void propagate_error();
    void propagate_covariance();
    void propagate_state();
    void get_mean_pose(Eigen::Matrix<double, 6, 1> &mean_pose);
    void get_cov_pose(Eigen::Matrix<double, 6, 6> &cov_pose);
    void publish_odom();
    void publish_bias();

    void update_meas_mean(Eigen::Matrix<double, 6, 1> &mean_meas);
    void update_meas_cov(Eigen::Matrix<double, 6, 6> &cov_meas);
    void update_meas_flag();
    void update_error();
    void update_state();
    void reset_error();

private:

    // nominal states
    tf::Vector3    _velocity;
    tf::Matrix3x3  _rotation;
    tf::Quaternion _quaternion;
    tf::Vector3    _position;
    tf::Vector3    _bias_acc;
    tf::Vector3    _bias_gyr;

    // error states
    tf::Vector3   _d_velocity;
    tf::Vector3   _d_theta;
    tf::Matrix3x3 _d_rotation;
    tf::Vector3   _d_position;
    tf::Vector3   _d_bias_acc;
    tf::Vector3   _d_bias_gyr;

    // imu measurements
    tf::Vector3    _imu_acceleration;
    tf::Vector3    _imu_angular_velocity;
    tf::Quaternion _imu_orientation;

    // Jacobian matrices
    Eigen::Matrix<double, 15, 15> _Fx;
    Eigen::Matrix<double, 15, 12> _Fn;

    // covariance matrix
    Eigen::Matrix<double, 15, 15> _Sigma;
    Eigen::Matrix<double, 12, 12> _Q;

    // gravity
    double _g;
    tf::Vector3 _gravity;

    // time relatives
    ros::Time _imu_time;
    double _dt, _imu_freq;
    bool _init_time;

    // noise params
    double _sigma_acc, _sigma_gyr;
    double _sigma_bias_acc, _sigma_bias_gyr;

    // initialization params
    double _init_bias_acc_x, _init_bias_acc_y, _init_bias_acc_z;

    // subscriber and publisher
    ros::Subscriber _imu_sub;
    ros::Publisher  _odom_pub, _bias_pub;

    // odometry measurements
    tf::Vector3    _m_theta;
    tf::Vector3    _m_position;
    bool _got_measurements;

    Eigen::Matrix<double, 6, 6> _m_pose_sigma;

    // a queue to smooth imu accleration measurements
    std::vector<geometry_msgs::Vector3> _acc_queue;
    int _acc_queue_size;
    int _acc_queue_count;

};

#endif // IMUODOM_H
