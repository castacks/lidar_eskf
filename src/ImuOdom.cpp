#include "lidar_eskf/ImuOdom.h"


inline tf::Matrix3x3 vec_to_rot(tf::Vector3 w) {
    tf::Matrix3x3 R;
    R.setRPY(w.x(), w.y(), w.z());

    return R;
}

inline tf::Matrix3x3 skew(tf::Vector3 w) {
    tf::Matrix3x3 R;
    R.setValue(   0.0, -w.z(),  w.y(),
                w.z(),    0.0, -w.x(),
               -w.y(),  w.x(),    0.0);
    return R;
}

ImuOdom::ImuOdom(ros::NodeHandle &nh) {

    nh.param("imu_frequency",           _imu_freq,         50.0);
    nh.param("sigma_acceleration",      _sigma_acc,        0.1);
    nh.param("sigma_gyroscop",          _sigma_gyr,        0.001);
    nh.param("sigma_acceleration_bias", _sigma_bias_acc,   0.00001);
    nh.param("sigma_gyroscope_bias",    _sigma_bias_gyr,   0.000001);
    nh.param("gravity",                 _g,                9.82);
    nh.param("init_bias_acc_x",         _init_bias_acc_x,  0.0);
    nh.param("init_bias_acc_y",         _init_bias_acc_y,  0.0);
    nh.param("init_bias_acc_z",         _init_bias_acc_z,  0.0);

    // initialize nomial states
    _velocity.setZero();
    _rotation.setIdentity();
    _quaternion.setRPY(0.0, 0.0, 0.0);
    _position.setZero();
    _bias_acc.setValue(_init_bias_acc_x, _init_bias_acc_y, _init_bias_acc_z);
    _bias_gyr.setZero();

    // initialize error states
    _d_velocity.setZero();
    _d_theta.setZero();
    _d_position.setZero();
    _d_bias_acc.setZero();
    _d_bias_gyr.setZero();

    // initialize measurements
    _imu_acceleration.setZero();
    _imu_angular_velocity.setZero();
    _imu_orientation.setRPY(0.0,0.0,0.0);

    // initialize Jacobian matrix;
    _Fx.setZero();
    _Fn.setZero();

    // initialize covariance matrix
    _Sigma.setZero();
    _Q.setZero();

    // gravity
    _gravity.setValue(0.0, 0.0, _g);

    // time relatives
    _init_time = true;

    // subscriber and publisher
    _imu_sub = nh.subscribe("/imu", 50, &ImuOdom::imu_callback, this);
    _imu_odom_pub = nh.advertise<nav_msgs::Odometry>("imu_odom", 50, true);
}

void ImuOdom::imu_callback(const sensor_msgs::Imu &msg) {
    update_time(msg);
    update_measurements(msg);

    propagate_state();
    propagate_error();
    propagate_covariance();

    publish_odom();
}

void ImuOdom::update_time(const sensor_msgs::Imu &msg) {

    if(_init_time) {
        _dt = 1.0 / _imu_freq;
        _init_time = false;
    }
    else {
        _dt = msg.header.stamp.toSec() - _imu_time.toSec();
    }
    _imu_time = msg.header.stamp;
}

void ImuOdom::update_measurements(const sensor_msgs::Imu &msg) {
    _imu_acceleration.setX(msg.linear_acceleration.x);
    _imu_acceleration.setY(msg.linear_acceleration.y);
    _imu_acceleration.setZ(msg.linear_acceleration.z);

    _imu_angular_velocity.setX(msg.angular_velocity.x);
    _imu_angular_velocity.setY(msg.angular_velocity.y);
    _imu_angular_velocity.setZ(msg.angular_velocity.z);

    _imu_orientation.setX(msg.orientation.x);
    _imu_orientation.setY(msg.orientation.y);
    _imu_orientation.setZ(msg.orientation.z);
    _imu_orientation.setW(msg.orientation.w);
}

void ImuOdom::propagate_state() {
    tf::Vector3   velocity;
    tf::Matrix3x3 rotation;
    tf::Vector3   position;
    tf::Vector3   bias_acc;
    tf::Vector3   bias_gyr;

    // system transition function for nominal state
    velocity = _velocity + (_rotation * (_imu_acceleration - _bias_acc) + _gravity ) * _dt;
    rotation = _rotation * vec_to_rot((_imu_angular_velocity - _bias_gyr) * _dt);
    position = _position + _velocity * _dt + 0.5 * (_rotation * (_imu_acceleration - _bias_acc) + _gravity) * _dt * _dt;
    bias_acc = _bias_acc;
    bias_gyr = _bias_gyr;

    // update norminal state to the next step
    _velocity = velocity;
    _rotation = rotation;
    _position = position;
    _bias_acc = bias_acc;
    _bias_gyr = bias_gyr;

    _rotation.getRotation(_quaternion);
}

void ImuOdom::propagate_error() {

    // system transition function for error state
    // this is not necessary because it is always zero with out measurement update
}

void ImuOdom::propagate_covariance() {
    // compute jacobian
    Eigen::Matrix<double, 3, 3> I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Z = Eigen::MatrixXd::Zero(3, 3);

    Eigen::Matrix<double, 3, 3> R, R_1, R_2;
    tf::matrixTFToEigen(_rotation, R);
    tf::matrixTFToEigen(skew(_imu_acceleration - _bias_acc), R_1);
    tf::matrixTFToEigen(vec_to_rot((_imu_angular_velocity - _bias_gyr) * _dt), R_2);

    _Fx <<     I,        -R*R_1*_dt,   Z,   -R*_dt,        Z,
               Z,   R_2.transpose(),   Z,        Z,   -I*_dt,
           I*_dt,                 Z,   I,        Z,        Z,
               Z,                 Z,   Z,        I,        Z,
               Z,                 Z,   Z,        Z,        I;

    _Fn << R, Z, Z, Z,
           Z, I, Z, Z,
           Z, Z, Z, Z,
           Z, Z, I, Z,
           Z, Z, Z, I;

    _Q << pow(_sigma_acc * _dt, 2.0) * I, Z, Z, Z,
          Z, pow(_sigma_gyr * _dt, 2.0) * I, Z, Z,
          Z, Z, pow(_sigma_bias_acc * _dt, 2.0) * I, Z,
          Z, Z, Z, pow(_sigma_bias_gyr * _dt, 2.0) * I;

    // update covariance
    _Sigma = _Fx * _Sigma * _Fx.transpose() + _Fn * _Q * _Fn.transpose();
}

void ImuOdom::publish_odom() {
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = _imu_time;

    // fill in pose and twist
    msg.pose.pose.position.x = _position.x();
    msg.pose.pose.position.y = _position.y();
    msg.pose.pose.position.z = _position.z();
    msg.pose.pose.orientation.x = _quaternion.x();
    msg.pose.pose.orientation.y = _quaternion.y();
    msg.pose.pose.orientation.z = _quaternion.z();
    msg.pose.pose.orientation.w = _quaternion.w();

    msg.twist.twist.angular.x = _imu_angular_velocity.x();
    msg.twist.twist.angular.y = _imu_angular_velocity.y();
    msg.twist.twist.angular.z = _imu_angular_velocity.z();
    msg.twist.twist.linear.x = _velocity.x();
    msg.twist.twist.linear.y = _velocity.y();
    msg.twist.twist.linear.z = _velocity.z();

    // fill in covariance
    Eigen::Matrix<double, 6, 6> pose_sigma, twist_sigma;
    Eigen::Matrix<double, 3, 3> R;
    tf::matrixTFToEigen(_rotation, R);

    pose_sigma <<     _Sigma.block<3,3>(6,6),     _Sigma.block<3,3>(3,6) * R.transpose(),
                  R * _Sigma.block<3,3>(6,3), R * _Sigma.block<3,3>(3,3) * R.transpose();

    twist_sigma <<        _Sigma.block<3,3>(0,0),                      Eigen::MatrixXd::Zero(3,3) * R.transpose(),
                  R * Eigen::MatrixXd::Zero(3,3), R * _sigma_gyr * Eigen::MatrixXd::Identity(3,3) * R.transpose();

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            msg.pose.covariance[6*i+j] = pose_sigma(i, j);
            msg.twist.covariance[6*i+j] = twist_sigma(i, j);
        }
    }

    // publish message
    _imu_odom_pub.publish(msg);
}
