/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "lidar_eskf/eskf.h"

ros::Publisher fake_meas_pub;
ros::Subscriber imu_odom_sub;
ros::Subscriber fake_odom_sub;

tf::Vector3 position;
tf::Vector3 velocity;
tf::Matrix3x3 rotation;
tf::Quaternion quaternion;

tf::Vector3    d_velocity;
tf::Vector3    d_position;
tf::Matrix3x3  d_rotation;
tf::Quaternion d_quaternion;

double pos_sigma, rot_sigma;

void imu_odom_callback(const nav_msgs::Odometry &msg) {
    position.setX(msg.pose.pose.position.x);
    position.setY(msg.pose.pose.position.y);
    position.setZ(msg.pose.pose.position.z);

    velocity.setX(msg.twist.twist.linear.x);
    velocity.setY(msg.twist.twist.linear.y);
    velocity.setZ(msg.twist.twist.linear.z);

    quaternion.setX(msg.pose.pose.orientation.x);
    quaternion.setY(msg.pose.pose.orientation.y);
    quaternion.setZ(msg.pose.pose.orientation.z);
    quaternion.setW(msg.pose.pose.orientation.w);

    rotation.setRotation(quaternion);
}
void fake_odom_callback(const nav_msgs::Odometry &msg) {
    tf::Vector3 fake_position;
    tf::Matrix3x3 fake_rotation;
    tf::Quaternion fake_quaternion;

    tf::pointMsgToTF(msg.pose.pose.position, fake_position);
    tf::quaternionMsgToTF(msg.pose.pose.orientation, fake_quaternion);
    fake_rotation.setRotation(fake_quaternion);

    d_position = fake_position - position;
    d_rotation = rotation.transpose() * fake_rotation;
    d_rotation.getRotation(d_quaternion);

}
void publish_fake_measure(const ros::TimerEvent&) {

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    msg.pose.pose.position.x = d_position.x();
    msg.pose.pose.position.y = d_position.y();
    msg.pose.pose.position.z = d_position.z();

    msg.pose.pose.orientation.x = d_quaternion.x();
    msg.pose.pose.orientation.y = d_quaternion.y();
    msg.pose.pose.orientation.z = d_quaternion.z();
    msg.pose.pose.orientation.w = d_quaternion.w();

    msg.pose.covariance[0]  = pow(pos_sigma, 2.0);
    msg.pose.covariance[7]  = pow(pos_sigma, 2.0);
    msg.pose.covariance[14] = pow(pos_sigma, 2.0);
    msg.pose.covariance[21] = pow(rot_sigma, 2.0);
    msg.pose.covariance[28] = pow(rot_sigma, 2.0);
    msg.pose.covariance[35] = pow(rot_sigma, 2.0);

    fake_meas_pub.publish(msg);
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "eskf_test");
    ros::NodeHandle n("~");

    n.param("position_sigma", pos_sigma, 0.1);
    n.param("rotation_sigma", rot_sigma, 0.1);

    position.setZero();
    velocity.setZero();
    rotation.setIdentity();
    quaternion.setRPY(0.0,0.0,0.0);

    fake_meas_pub = n.advertise<nav_msgs::Odometry>("/meas", 50, true);
    imu_odom_sub  = n.subscribe("/eskf_test/odom", 50, &imu_odom_callback);
    fake_odom_sub = n.subscribe("/dji_sim/odometry", 50, &fake_odom_callback);

    ros::Timer timer = n.createTimer(ros::Duration(0.03), publish_fake_measure);
    ESKF eskf(n);

    ros::spin();
}
