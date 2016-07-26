#ifndef IMUODOM_H
#define IMUODOM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>

class ImuOdom {
public:
    ImuOdom(ros::NodeHandle &nh);
    ~ImuOdom() {}

    void imu_callback(const sensor_msgs::Imu &msg);
    void update_time(const sensor_msgs::Imu &msg);
    void update_measurements(const sensor_msgs::Imu &msg);
    void propagate_state();
    void propagate_error();
    void propagate_covariance();
    void publish_odom();
private:

    // nominal states
    tf::Vector3    _velocity;
    tf::Matrix3x3  _rotation;
    tf::Quaternion _quaternion;
    tf::Vector3    _position;
    tf::Vector3    _bias_acc;
    tf::Vector3    _bias_gyr;

    // error states
    tf::Vector3 _d_velocity;
    tf::Vector3 _d_theta;
    tf::Vector3 _d_position;
    tf::Vector3 _d_bias_acc;
    tf::Vector3 _d_bias_gyr;

    // imu measurements
    tf::Vector3 _imu_acceleration;
    tf::Vector3 _imu_angular_velocity;
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
    ros::Publisher  _imu_odom_pub;

};

#endif // IMUODOM_H
