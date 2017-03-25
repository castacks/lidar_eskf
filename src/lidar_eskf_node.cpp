#include <lidar_eskf/eskf.h>
#include <lidar_eskf/gpf.h>

int main(int argc, char **argv) {
    // initialize ros
    ros::init(argc, argv, "lidar_eskf_node");
    ros::NodeHandle n("~");

    boost::shared_ptr<DistMap> map_ptr = boost::shared_ptr<DistMap>(new DistMap(n));
    sleep(1);
    boost::shared_ptr<GPF> gpf_ptr = boost::shared_ptr<GPF>(new GPF(n, map_ptr));
    ros::spin();
    return 0;

}
