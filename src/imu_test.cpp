#include "lidar_eskf/ImuOdom.h"

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "imu_test");
    ros::NodeHandle n("~");

    ImuOdom imu(n);

    ros::spin();
}
