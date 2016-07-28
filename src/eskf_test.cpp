#include "lidar_eskf/eskf.h"

ros::Publisher fake_odom_pub;
ros::Subscriber imu_odom_sub;

tf::Vector3 position;
tf::Vector3 velocity;
tf::Matrix3x3 rotation;
tf::Quaternion quaternion;

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

void publish_fake_odom(const ros::TimerEvent&) {
    tf::Vector3    d_velocity;
    tf::Vector3    d_position;
    tf::Matrix3x3  d_rotation;
    tf::Quaternion d_quaternion;

    d_velocity = -velocity;
    d_position = -position;

    double r, p, y;
    d_rotation.setRotation(quaternion);
    d_rotation.getRPY(r, p, y);
    d_rotation.setRPY(-r, -p, -y);
    d_rotation.getRotation(d_quaternion);

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

    fake_odom_pub.publish(msg);
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

    fake_odom_pub = n.advertise<nav_msgs::Odometry>("fake_measurements", 50, true);
    imu_odom_sub = n.subscribe("imu_odom", 50, &imu_odom_callback);

    ros::Timer timer = n.createTimer(ros::Duration(0.025), publish_fake_odom);
    ESKF eskf(n);

    ros::spin();
}
