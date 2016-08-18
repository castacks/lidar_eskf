#include "lidar_eskf/eskf.h"

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

ESKF::ESKF(ros::NodeHandle &nh) {

    nh.param("imu_frequency",           _imu_freq,         50.0);
    nh.param("sigma_acceleration",      _sigma_acc,        0.1);
    nh.param("sigma_gyroscop",          _sigma_gyr,        0.01);
    nh.param("sigma_acceleration_bias", _sigma_bias_acc,   0.0001);
    nh.param("sigma_gyroscope_bias",    _sigma_bias_gyr,   0.00001);
    nh.param("gravity",                 _g,                9.82);
    nh.param("init_bias_acc_x",         _init_bias_acc_x,  0.0);
    nh.param("init_bias_acc_y",         _init_bias_acc_y,  0.0);
    nh.param("init_bias_acc_z",         _init_bias_acc_z,  0.0);
    nh.param("acc_queue_size",          _acc_queue_size,   5);

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
    _d_rotation.setIdentity();
    _d_position.setZero();
    _d_bias_acc.setZero();
    _d_bias_gyr.setZero();

    // initialize imu
    _imu_acceleration.setZero();
    _imu_angular_velocity.setZero();
    _imu_orientation.setRPY(0.0,0.0,0.0);

    // initialize measurements
    _m_velocity.setZero();
    _m_rotation.setIdentity();
    _m_quaternion.setRPY(0.0, 0.0, 0.0);
    _m_position.setZero();
    _m_theta.setZero();
    _got_measurements = false;

    // initialize Jacobian matrix;
    _Fx.setZero();
    _Fn.setZero();

    // initialize covariance matrix
    _Sigma.setZero();
//    _Sigma.block<3,3>(3,3) = 0.01 * Eigen::MatrixXd::Identity(3, 3);
    _Q.setZero();

    // gravity
    _gravity.setValue(0.0, 0.0, _g);

    // time relatives
    _init_time = true;

    // subscriber and publisher
    _imu_sub = nh.subscribe("/imu", 50, &ESKF::imu_callback, this);
    _meas_sub = nh.subscribe("/meas", 50, &ESKF::measurement_callback, this);
    _odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50, true);
    _bias_pub = nh.advertise<geometry_msgs::TwistStamped>("bias", 50, true);
    // acc queue
    _acc_queue_count = 0;
}

void ESKF::imu_callback(const sensor_msgs::Imu &msg) {
    update_time(msg);
    update_imu(msg);

    propagate_state();
    propagate_error();
    propagate_covariance();

    // when a new measurement is available, update odometry
    if(_got_measurements) {
        // do measurements update
        update_error();
        update_state();
        reset_error();
        publish_bias();
        _got_measurements = false;
    }

    publish_odom();
}

void ESKF::update_time(const sensor_msgs::Imu &msg) {

    if(_init_time) {
        _dt = 1.0 / _imu_freq;
        _init_time = false;
    }
    else {
        _dt = msg.header.stamp.toSec() - _imu_time.toSec();
    }
    _imu_time = msg.header.stamp;
}

void ESKF::update_imu(const sensor_msgs::Imu &msg) {

    // stacking into a queue
    if(_acc_queue_count < _acc_queue_size) {
        _acc_queue.push_back(msg.linear_acceleration);
        tf::vector3MsgToTF(msg.linear_acceleration, _imu_acceleration);
    }
    else {
        _acc_queue[_acc_queue_count%_acc_queue_size] = msg.linear_acceleration;

        tf::Vector3 acc_avg;
        acc_avg.setZero();
        for(int i=0; i<_acc_queue_size; i++) {
            acc_avg[0] += _acc_queue[i].x / _acc_queue_size;
            acc_avg[1] += _acc_queue[i].y / _acc_queue_size;
            acc_avg[2] += _acc_queue[i].z / _acc_queue_size;
        }
        _imu_acceleration = acc_avg;
    }
    _acc_queue_count++;

    _imu_angular_velocity.setX(msg.angular_velocity.x);
    _imu_angular_velocity.setY(msg.angular_velocity.y);
    _imu_angular_velocity.setZ(msg.angular_velocity.z);

    _imu_orientation.setX(msg.orientation.x);
    _imu_orientation.setY(msg.orientation.y);
    _imu_orientation.setZ(msg.orientation.z);
    _imu_orientation.setW(msg.orientation.w);

    tf::Matrix3x3 imu_rotation;
    tf::Vector3 grav(0.0, 0.0, -9.82);
    imu_rotation.setRotation(_imu_orientation);

    _imu_acceleration -= imu_rotation.transpose() * grav;
//    _imu_acceleration.setZero();
    ROS_INFO("ESKF: acceleration: \t %0.4f \t %0.4f \t %0.4f \t", _imu_acceleration[0], _imu_acceleration[1], _imu_acceleration[2]);

}

void ESKF::propagate_state() {
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

void ESKF::propagate_error() {

    // system transition function for error state
    // this is not necessary because it is always zero with out measurement update
}

void ESKF::propagate_covariance() {
    // compute jacobian
    Eigen::Matrix<double, 3, 3> I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Z = Eigen::MatrixXd::Zero(3, 3);

    Eigen::Matrix<double, 3, 3> R, R_1, R_2;
    tf::matrixTFToEigen(_rotation, R);
    tf::matrixTFToEigen(skew(_imu_acceleration - _bias_acc), R_1);
    tf::matrixTFToEigen(vec_to_rot((_imu_angular_velocity - _bias_gyr) * _dt), R_2);

    _Fx <<     I,      Z,       -R*R_1*_dt,   -R*_dt,        Z,
           I*_dt,      I,                Z,        Z,        Z,
               Z,      Z,  R_2.transpose(),        Z,   -I*_dt,
               Z,      Z,                Z,        I,        Z,
               Z,      Z,                Z,        Z,        I;

    _Fn << R, Z, Z, Z,
           Z, Z, Z, Z,
           Z, I, Z, Z,
           Z, Z, I, Z,
           Z, Z, Z, I;

    _Q << pow(_sigma_acc, 2.0) * I, Z, Z, Z,
          Z, pow(_sigma_gyr, 2.0) * I, Z, Z,
          Z, Z, pow(_sigma_bias_acc, 2.0) * I, Z,
          Z, Z, Z, pow(_sigma_bias_gyr, 2.0) * I;

    // update covariance
    _Sigma = _Fx * _Sigma * _Fx.transpose() + _Fn * _Q * _Fn.transpose();
}

void ESKF::get_mean_pose(Eigen::Matrix<double, 6, 1> &mean_pose) {
    mean_pose[0] = _position.x();
    mean_pose[1] = _position.y();
    mean_pose[2] = _position.z();

    _rotation.getRPY(mean_pose[3], mean_pose[4], mean_pose[5]);
}

void ESKF::get_cov_pose(Eigen::Matrix<double, 6, 6> &cov_pose) {
    cov_pose = _Sigma.block<6,6>(3,3);
}

void ESKF::publish_odom() {
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

    pose_sigma << _Sigma.block<6,6>(3,3);

    twist_sigma << _Sigma.block<3,3>(0,0),      Eigen::MatrixXd::Zero(3,3),
                   Eigen::MatrixXd::Zero(3,3), _sigma_gyr * Eigen::MatrixXd::Identity(3,3);

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            msg.pose.covariance[6*i+j] = pose_sigma(i, j);
            msg.twist.covariance[6*i+j] = twist_sigma(i, j);
        }
    }

    // publish message
    _odom_pub.publish(msg);
}

void ESKF::publish_bias() {
    geometry_msgs::TwistStamped msg;

    msg.header.frame_id = "world";
    msg.header.stamp = _imu_time;

    msg.twist.linear.x = _bias_acc.x();
    msg.twist.linear.y = _bias_acc.y();
    msg.twist.linear.z = _bias_acc.z();

    msg.twist.angular.x = _bias_gyr.x();
    msg.twist.angular.y = _bias_gyr.y();
    msg.twist.angular.z = _bias_gyr.z();

    _bias_pub.publish(msg);
}

void ESKF::measurement_callback(const nav_msgs::Odometry &msg) {
    _m_velocity.setX(msg.twist.twist.linear.x);
    _m_velocity.setY(msg.twist.twist.linear.y);
    _m_velocity.setZ(msg.twist.twist.linear.z);

    _m_quaternion.setW(msg.pose.pose.orientation.w);
    _m_quaternion.setX(msg.pose.pose.orientation.x);
    _m_quaternion.setY(msg.pose.pose.orientation.y);
    _m_quaternion.setZ(msg.pose.pose.orientation.z);

    _m_rotation.setRotation(_m_quaternion);
    _m_rotation.getRPY(_m_theta[0], _m_theta[1], _m_theta[2]);

    _m_position.setX(msg.pose.pose.position.x);
    _m_position.setY(msg.pose.pose.position.y);
    _m_position.setZ(msg.pose.pose.position.z);

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            _m_pose_sigma(i, j) = msg.pose.covariance[6*i+j];
            _m_twist_sigma(i, j) = msg.twist.covariance[6*i+j];
        }
    }

    _got_measurements = true;
}

void ESKF::update_mean_meas(Eigen::Matrix<double, 6, 1> &mean_meas) {
    _m_position.setValue(mean_meas[0],mean_meas[1],mean_meas[2]);
    _m_quaternion.setRPY(mean_meas[3],mean_meas[4],mean_meas[5]);
    _m_rotation.setRotation(_m_quaternion);
}

void ESKF::update_cov_meas(Eigen::Matrix<double, 6, 6> &cov_meas) {
    _m_pose_sigma = cov_meas;
}

void ESKF::update_meas_flag() {
    _got_measurements = true;
}
void ESKF::update_error() {
    // assume only pose measurement is used
    Eigen::Matrix<double, 6, 15> H;
    Eigen::Matrix<double, 3, 3> I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Z = Eigen::MatrixXd::Zero(3, 3);
    H << Z, I, Z, Z, Z,
         Z, Z, I, Z, Z;

    // measurements
    Eigen::Matrix<double, 6, 1> y;
    y[0] = _m_position.x(); y[1] = _m_position.y(); y[2] = _m_position.z();
    y[3] = _m_theta.x();    y[4] = _m_theta.y();    y[5] = _m_theta.z();

    // kalman gain matrix
    Eigen::Matrix<double, 15, 6> K;
    K = _Sigma * H.transpose() * (H * _Sigma * H.transpose() + 3.0 * _m_pose_sigma).inverse();

    // update error
    Eigen::Matrix<double, 15, 1> x;
    x = K * y;

    _d_velocity.setValue(x[0],  x[1],  x[2]);
    _d_position.setValue(x[3],  x[4],  x[5]);
    _d_theta.setValue(   x[6],  x[7],  x[8]);
    _d_bias_acc.setValue(x[9],  x[10], x[11]);
    _d_bias_gyr.setValue(x[12], x[13], x[14]);

    _d_rotation.setRPY(_d_theta.x(),_d_theta.y(),_d_theta.z());

    // update covariance
    Eigen::Matrix<double, 15, 15> M;
    M = Eigen::MatrixXd::Identity(15,15) - K*H;
    _Sigma = M * _Sigma;// * M.transpose() + K * _m_pose_sigma * K.transpose();

}

void ESKF::update_state() {
    _velocity += _d_velocity;
    _position += _d_position;
    _rotation *= _d_rotation;
    _bias_acc += _d_bias_acc;
    _bias_gyr += _d_bias_gyr;

    _rotation.getRotation(_quaternion);
}

void ESKF::reset_error() {
    _d_velocity.setZero();
    _d_position.setZero();
    _d_theta.setZero();
    _d_rotation.setIdentity();
    _d_bias_acc.setZero();
    _d_bias_gyr.setZero();
}
