/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
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

typedef pcl::PointCloud<pcl::PointXYZ>  PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZ                   Point;
typedef pcl::PointXYZI                  PointI;

class BagSaver {
public:
    BagSaver(ros::NodeHandle &nh);
    ~BagSaver(){}

    void imu_callback(const sensor_msgs::Imu &msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg);
    void rotate_pcd();
    void save_pcd();
    void save_bt();

    PointCloudI map;
    PointCloudI map_tf;

    std::string pcd_file;
    std::string robot_frame;
    std::string imu_frame;
    std::string bt_file_name;
    double record_time;
    double range_max;
    double range_min;
    double octomap_resolution;
    double pcd_resolution;
    bool save_pcd_true;
    bool imu_distort;
    bool save_bt_true;
    tf::StampedTransform imu_frame_to_base_frame;


    ros::Subscriber imu_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber cloud_sub;

    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    std::vector<double> roll, pitch, yaw;

};

BagSaver::BagSaver(ros::NodeHandle &nh) {

    nh.param("pcd_file_name",         pcd_file,              std::string("map.pcd"));
    nh.param("record_time",           record_time,           60.0);
    nh.param("max_range",             range_max,             100.0);
    nh.param("min_range",             range_min,             0.5);
    nh.param("robot_frame",   	      robot_frame,           std::string("/base_frame"));
    nh.param("imu_distort",           imu_distort,           false);
    nh.param("octomap_resolution",    octomap_resolution,    0.1);
    nh.param("bt_file_name",     	  bt_file_name,          std::string("map.bt"));
    nh.param("save_pcd_true",		  save_pcd_true,         false);
    nh.param("save_bt_true",          save_bt_true,          true);
    nh.param("pcd_resolution",        pcd_resolution,        0.1);

    imu_sub = nh.subscribe("/imu", 100, &BagSaver::imu_callback, this);
    scan_sub = nh.subscribe("/scan", 10, &BagSaver::laserscan_callback, this);
    cloud_sub = nh.subscribe("/cloud", 10, &BagSaver::pointcloud_callback, this);

}

void BagSaver::imu_callback(const sensor_msgs::Imu &msg) {

    imu_frame = msg.header.frame_id;

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
    yaw.push_back(y);

    //ROS_INFO_ONCE("IMU callback.");

}

void BagSaver::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointCloudI scan, scan_temp;
    pcl::fromROSMsg(*msg, scan_temp);
    if(!listener.waitForTransform(
            msg->header.frame_id,
            robot_frame,
            ros::Time::now(),
            ros::Duration(0.1))){
        ROS_WARN("Can not find transform from laser_frame to robot_frame"); 
        return;
    }
    listener.lookupTransform(imu_frame, robot_frame, ros::Time::now(), imu_frame_to_base_frame);

    pcl_ros::transformPointCloud(robot_frame, scan_temp, scan, listener);

    // stacking scans
    for(int i=0; i<scan.size(); i++) {
        pcl::PointXYZI p = scan[i];
        double d = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if(d > range_min && d < range_max) {
                map.push_back(p);
        }
    }
    //ROS_INFO_THROTTLE(1.0, "Cloud callback");
}

void BagSaver::laserscan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    PointCloudI scan;
    PointCloudI::Ptr scan_ptr(new PointCloudI(scan));

    if(!listener.waitForTransform(
            msg->header.frame_id,
            robot_frame,
            msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2::Ptr scan_cloud_ptr = sensor_msgs::PointCloud2::Ptr (new sensor_msgs::PointCloud2);
    projector.transformLaserScanToPointCloud(robot_frame,*msg,*scan_cloud_ptr,listener);
    
    pointcloud_callback(scan_cloud_ptr);

}

void BagSaver::rotate_pcd() {

    double mean_roll  = std::accumulate(roll.begin(),  roll.end(),  0.0) / roll.size();
    double mean_pitch = std::accumulate(pitch.begin(), pitch.end(), 0.0) / pitch.size();
    double mean_yaw   = std::accumulate(yaw.begin(),   yaw.end(),   0.0) / yaw.size();

    tf::Matrix3x3 imu_rotation;
    imu_rotation.setRPY(mean_roll, mean_pitch, 0.0);
    tf::Matrix3x3 base_rotation;
    base_rotation.setRotation(imu_frame_to_base_frame.getRotation());
    tf::Matrix3x3 ftpt_to_base_rotation = imu_rotation * base_rotation;
    double r, p, y;
    ftpt_to_base_rotation.getRPY(r,p,y);
    ftpt_to_base_rotation.setRPY(r,p,0.0);

    Eigen::Matrix3d rotation_world_to_imu;
    Eigen::Matrix3d rotation_imu_to_base;
    Eigen::Matrix3d rotation_ftpt_to_base;
    Eigen::Vector3d translation_world_to_imu;
    Eigen::Vector3d translation_imu_to_base;

    translation_world_to_imu.setZero();
    translation_imu_to_base << imu_frame_to_base_frame.getOrigin().x(),
			       imu_frame_to_base_frame.getOrigin().y(),
			       imu_frame_to_base_frame.getOrigin().z();


    tf::matrixTFToEigen(imu_rotation, rotation_world_to_imu);
    tf::matrixTFToEigen(base_rotation, rotation_imu_to_base);
    tf::matrixTFToEigen(ftpt_to_base_rotation, rotation_ftpt_to_base);
    Eigen::Matrix4d T1;
    Eigen::Matrix4d T2;
    Eigen::Matrix4d T;
    T1 << rotation_world_to_imu, translation_world_to_imu,
          0.0,0.0,0.0,1.0;
    T2 << rotation_imu_to_base, translation_imu_to_base,
          0.0,0.0,0.0,1.0;
    T = T1 * T2;
    T << rotation_ftpt_to_base, translation_world_to_imu,
      	 0.0,0.0,0.0,1.0;
    pcl::transformPointCloud(map, map_tf, T);
}

void BagSaver::save_pcd() {
	pcl::VoxelGrid<pcl::PointXYZI> sor;

	pcl::PointCloud<pcl::PointXYZI> map_downsampled;
	
	if(imu_distort == true) {
	    std::cout << "saving undistorted map" << std::endl;
	    sor.setInputCloud(map_tf.makeShared());
	} else {
	    std::cout << "saving raw map" << std::endl;
	    sor.setInputCloud(map.makeShared());
	}
	sor.setLeafSize(float(pcd_resolution), float(pcd_resolution), float(pcd_resolution));
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
		  const PointI& p = map_tf.points[pt_idx];
		  if (!std::isnan(p.z))
		    octomap_cloud.push_back(p.x, p.y, p.z);
		}
	} else {
		for (unsigned int pt_idx = 0; pt_idx < map.points.size(); ++pt_idx)
		{
		  const PointI& p = map.points[pt_idx];
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

int  main (int argc, char** argv) {
     ros::init(argc, argv, "bag_to_pcd");
     ros::NodeHandle n("~");

     BagSaver saver(n);

     ros::Rate rate(40);
     int cnt = 0;
     while(ros::ok() && cnt <= saver.record_time) {

         ROS_INFO_STREAM_THROTTLE(1.0, "time = " << cnt++ << "/" << saver.record_time << "\t");

         ros::spinOnce();
         rate.sleep();
     }
    
     saver.rotate_pcd();  
     if(saver.save_pcd_true) {
         saver.save_pcd();
     }
     if(saver.save_bt_true) {
         ROS_INFO("Start to convert to Octomap, please wait ...");
         saver.save_bt();
     }
     return 0;
}
