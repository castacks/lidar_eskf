#include <lidar_eskf/eskf.h>
#include <lidar_eskf/gpf.h>
#include <std_msgs/Bool.h>

bool reset_triggered = false;
geometry_msgs::Pose* pose_msg;

void reset_callback(const geometry_msgs::Pose &msg)
{
	reset_triggered = true;
	pose_msg = new geometry_msgs::Pose();
	pose_msg->position = msg.position;
	pose_msg->orientation = msg.orientation;
}


int main(int argc, char **argv) {
    // initialize ros
    ros::init(argc, argv, "lidar_eskf_node");
    ros::NodeHandle n("~");
    
    boost::shared_ptr<DistMap> map_ptr = boost::shared_ptr<DistMap>(new DistMap(n));
    sleep(1);
    GPF* gpf_ptr = new GPF(n, map_ptr,0,0,0);
    
    ros::Subscriber reset_sub = n.subscribe("/dji_sim/reset", 100, &reset_callback);

    // ros::Publisher nearest_pub = n.advertise<geometry_msgs::PointStamped>("/dji_sim/nearest_obstacle", 100);

    ros::Rate r(40);
    while(ros::ok())
    {
        if(reset_triggered){
            ROS_INFO("RESETTING GPF POINTER");
            delete(gpf_ptr);
            std::cout << pose_msg->position << std::endl;
            gpf_ptr = new GPF(n, map_ptr,pose_msg->position.x,-pose_msg->position.y,-pose_msg->position.z);
            reset_triggered = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;


}
