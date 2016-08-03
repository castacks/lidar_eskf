#include <lidar_eskf/eskf.h>
#include <lidar_eskf/gpf.h>

int main(int argc, char **argv) {
    // initialize ros
    ros::init(argc, argv, "lidar_eskf_node");
    ros::NodeHandle n("~");

    DistMap map(n);
    ESKF eskf(n);
    GPF gpf(n, map);

}
