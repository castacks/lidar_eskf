#include "lidar_eskf/eskf.h"

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "eskf_test");
    ros::NodeHandle n("~");

    ESKF eskf(n);

    ros::spin();
}
