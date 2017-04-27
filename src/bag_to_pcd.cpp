#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ                  PointT;

class BagSaver {
public:
    BagSaver(ros::NodeHandle nh);
    ~BagSaver(){}

    void imu_callback(const sensor_msgs::Imu &msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
    void rotate_pcd();
    void save_pcd();
    void save_bt();

    PointCloud map;
    PointCloud map_tf;

    std::string imu_topic;
    std::string laser_topic;
    std::string laser_type;
    std::string pcd_file;
    std::string target_frame;
    std::string bt_file_name;
    double laser_freq;
    double record_time;
    double range_max;
    double range_min;
    double octomap_resolution;
    bool save_pcd_true;
    bool imu_distort;

    ros::Subscriber imu_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber cloud_sub;
    bool stop;

    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    std::vector<double> roll, pitch;

};

BagSaver::BagSaver(ros::NodeHandle nh) {
    nh.getParam("laser_topic", 		laser_topic);
    nh.getParam("pcd_file_name", 	pcd_file);
    nh.getParam("laser_freq", 		laser_freq);
    nh.getParam("record_time", 		record_time);
    nh.param("max_range",               range_max, 100.0);
    nh.param("min_range",               range_min, 0.5);
    nh.getParam("laser_type", 		laser_type);
    nh.getParam("robot_frame",   	target_frame);
    nh.getParam("imu_distort",          imu_distort);
    nh.getParam("octomap_resolution",   octomap_resolution);
    nh.getParam("bt_file_name",     	bt_file_name);
	nh.getParam("save_pcd_true",		save_pcd_true);
	nh.getParam("imu_topic",            imu_topic);

    imu_sub = nh.subscribe(imu_topic, 100, &BagSaver::imu_callback, this);
    if(laser_type == "laserscan") {
        laser_sub = nh.subscribe(laser_topic, 10, &BagSaver::laserscan_callback, this);
    }
    else if(laser_type == "pointcloud") {
        cloud_sub = nh.subscribe(laser_topic, 10, &BagSaver::pointcloud_callback, this);
    }

    //scan_ptr.reset(&scan);
    stop = false;

}

void BagSaver::imu_callback(const sensor_msgs::Imu &msg) {
    tf::Quaternion q;

    q.setX(msg.orientation.x);
    q.setY(msg.orientation.y);
    q.setZ(msg.orientation.z);
    q.setW(msg.orientation.w);

    double r, p, y;
    tf::Matrix3x3 R;
    R.setRotation(q);

    R.getRPY(r, p, y);

    roll.push_back(r);
    pitch.push_back(p);
}

void BagSaver::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(stop == true) return;

    PointCloud scan, scan_temp;
    pcl::fromROSMsg(*msg, scan_temp);
    if(!listener.waitForTransform(
            msg->header.frame_id,
            target_frame,
            ros::Time::now(),
            ros::Duration(1.0))){
        return;
    }
    
    pcl_ros::transformPointCloud(target_frame, scan_temp, scan, listener);
    
    /*PointCloud::Ptr scan_ptr(new PointCloud(scan));
    
    // filtering
    PointCloud::Ptr xlim_cloud = PointCloud::Ptr (new PointCloud);
    PointCloud::Ptr ylim_cloud = PointCloud::Ptr (new PointCloud);
    PointCloud::Ptr zlim_cloud = PointCloud::Ptr (new PointCloud);
    
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(scan_ptr);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits (-range_lim, range_lim);
    pass_x.filter(*xlim_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xlim_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits (-range_lim, range_lim);
    pass_y.filter(*ylim_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(ylim_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits (-range_lim, range_lim);
    pass_z.filter(*zlim_cloud);

    scan_ptr.reset();
    scan_ptr = zlim_cloud;

    // truncate in a bounding range
    pcl::ConditionOr<pcl::PointXYZ>::Ptr rangeCond (new pcl::ConditionOr<pcl::PointXYZ> ());
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.5)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condRem;
    condRem.setCondition((rangeCond));
    condRem.setInputCloud(scan_ptr);
    condRem.setKeepOrganized(true);
    condRem.filter (*scan_ptr);*/


    // stacking scans
    //PointCloud cloud;
    for(int i=0; i<scan.size(); i++) {
        pcl::PointXYZ p = scan[i];
	double d = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	if(d > range_min && d < range_max) {
            map.push_back(p);
	}
    }
}

void BagSaver::laserscan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    if(stop == true) return;

    PointCloud scan;
    PointCloud::Ptr scan_ptr(new PointCloud(scan));

    if(!listener.waitForTransform(
            msg->header.frame_id,
            target_frame,
            msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2::Ptr scan_cloud_ptr = sensor_msgs::PointCloud2::Ptr (new sensor_msgs::PointCloud2);
    projector.transformLaserScanToPointCloud(target_frame,*msg,*scan_cloud_ptr,listener);
    
    pointcloud_callback(scan_cloud_ptr);

}

void BagSaver::rotate_pcd() {

    double mean_roll = std::accumulate(roll.begin(), roll.end(), 0.0) / roll.size();
    double mean_pitch = std::accumulate(pitch.begin(), pitch.end(), 0.0) / pitch.size();

    tf::Matrix3x3 R;
    R.setRPY(mean_roll, mean_pitch, 0.0);

    Eigen::Matrix3d rot;
    Eigen::Vector3d t;
    t.setZero();

    tf::matrixTFToEigen(R, rot);
    Eigen::Matrix4d T;
    T << rot,t,
         0.0,0.0,0.0,1.0;

    pcl::transformPointCloud(map, map_tf, T);
}
void BagSaver::save_pcd() {
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	pcl::PointCloud<pcl::PointXYZ> map_downsampled;
	
	if(imu_distort == true) {
	    sor.setInputCloud(map_tf.makeShared());
    	    //pcl::io::savePCDFileASCII (pcd_file, map_tf);
	} else {
	    sor.setInputCloud(map.makeShared());
	    //pcl::io::savePCDFileASCII (pcd_file, map);
	}
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(map_downsampled);
	pcl::io::savePCDFileASCII(pcd_file, map_downsampled);
}

void BagSaver::save_bt() {
    std::cout << "Octomap resolution is: " << octomap_resolution << std::endl;
    octomap::ColorOcTree tree(octomap_resolution);
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);
    octomap::pose6d frame_origin(0.0, 0, 0, 0, 0, 0);
    octomap::Pointcloud octomap_cloud;

    if(imu_distort == true) {
		for (unsigned int pt_idx = 0; pt_idx < map_tf.points.size(); ++pt_idx)
		{
		  const PointT& p = map_tf.points[pt_idx];
		  if (!std::isnan(p.z))
		    octomap_cloud.push_back(p.x, p.y, p.z);
		}
	} else {
		for (unsigned int pt_idx = 0; pt_idx < map.points.size(); ++pt_idx)
		{
		  const PointT& p = map.points[pt_idx];
		  if (!std::isnan(p.z))
		    octomap_cloud.push_back(p.x, p.y, p.z);
		}
	}

    // insert scan (only xyz considered, no colors)
    tree.insertPointCloud(octomap_cloud, sensor_origin, frame_origin);
    tree.updateInnerOccupancy();

    bool result = tree.writeBinary( bt_file_name ); //.bt
    if (result)
        std::cout << "Point cloud to octomap conversion is completed!" << std::endl;
    else
        std::cout << "Conversion failed!" << std::endl;
}

int  main (int argc, char** argv)
{
     ros::init(argc, argv, "bag_to_pcd");
     ros::NodeHandle n("bag_to_pcd");

     BagSaver saver(n);

     double tic = ros::Time::now().toSec();
     double toc = ros::Time::now().toSec();

     ros::Rate rate(saver.laser_freq);
     bool init = true;
     while(ros::ok() && (toc-tic) <= saver.record_time ) {

         if(init) {
             tic = ros::Time::now().toSec();
             init = false;
         }
         toc = ros::Time::now().toSec();
         ROS_INFO_STREAM_THROTTLE(1.0, "time = " << toc-tic << "/" << saver.record_time);
         ros::spinOnce();
         rate.sleep();
     }
    
     saver.stop = true;
     saver.rotate_pcd();  
     if(saver.save_pcd_true) {
	 	saver.save_pcd();
	 }
     ROS_INFO("Start to convert to Octomap, please wait ...");
     saver.save_bt();
     return (0);
}
