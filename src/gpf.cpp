/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "lidar_eskf/gpf.h"

template <typename T>
T add(T p1, T p2) {
    T p;
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.y = p1.z + p2.z;
    return p;
}

template <typename T>
T minus(T p1, T p2) {
    T p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    p.y = p1.z - p2.z;
    return p;
}

template <typename T, typename S>
T multiply(T p, S m) {
    T q;
    q.x = p.x * m;
    q.y = p.y * m;
    q.z = p.z * m;
    return q;
}

template <typename T>
double dot(T p1, T p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

template <typename T>
T cross(T p1, T p2) {
    T p;
    p.x = p1.y*p2.z - p2.y*p1.z;
    p.y = p2.x*p1.z - p1.x*p2.z;
    p.z = p1.x*p2.y - p2.x*p1.y;
    return p;
}

template <typename T>
double norm(T p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}
template <typename T>
T normalize(T p) {

    double n;
    n = norm(p);

    T np = multiply(p, 1/n);
    return np;
}
template <typename T>
std::vector<size_t> sort_index(const std::vector<T> v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}

GPF::GPF(ros::NodeHandle &nh, boost::shared_ptr<DistMap> map_ptr) : _map_ptr(map_ptr){


    // initialize particles pointer
    nh.param("cloud_sigma",             _ray_sigma,             1.0);
    nh.param("cloud_resolution",        _cloud_resol,           0.2);
    nh.param("set_size",                _set_size,              500);
    nh.param("cloud_range",             _cloud_range,           20.0);
    nh.param("laser_type",              _laser_type,            std::string("pointcloud"));
    nh.param("imu_to_laser_roll",       _imu_to_laser_roll,     0.0);
    nh.param("imu_to_laser_pitch",      _imu_to_laser_pitch,    0.0);
    nh.param("imu_to_laser_yaw",        _imu_to_laser_yaw,      0.0);
    nh.param("robot_frame",             _robot_frame,           std::string("/coax"));

    _mean_prior.setZero();
    _mean_sample.setZero();
    _mean_posterior.setZero();
    _mean_meas.setZero();

    Eigen::Matrix<double, 6, 1> sigma;
    sigma << 0.01, 0.01, 0.01, 0.005, 0.005, 0.005;
    _cov_prior = sigma.asDiagonal();
    _cov_sample.setZero();
    _cov_posterior.setZero();
    _cov_meas.setZero();

    if(_laser_type.compare("pointcloud") == 0) {
        _cloud_sub = nh.subscribe("cloud", 1, &GPF::cloud_callback, this);
    } else {
        _scan_sub = nh.subscribe("scan", 1, &GPF::scan_callback, this);
    }
    _cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("dsmp_cloud",1);
    _meas_pub = nh.advertise<nav_msgs::Odometry>("meas", 10);
    _pset_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
    _post_pub = nh.advertise<nav_msgs::Odometry>("posterior", 10);
    _path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

    _imu_to_laser_rotation.setRPY(_imu_to_laser_roll, _imu_to_laser_pitch, _imu_to_laser_yaw);

    _imu_to_laser_transform << _imu_to_laser_rotation[0][0], _imu_to_laser_rotation[0][1], _imu_to_laser_rotation[0][2], 0,
                               _imu_to_laser_rotation[1][0], _imu_to_laser_rotation[1][1], _imu_to_laser_rotation[1][2], 0,
                               _imu_to_laser_rotation[2][0], _imu_to_laser_rotation[2][1], _imu_to_laser_rotation[2][2], 0,
                                                          0,                            0,                            0, 1;
    // initialize eskf
    _eskf_ptr = boost::shared_ptr<ESKF> (new ESKF(nh));

    // initialize particle
    _particles_ptr = boost::shared_ptr<Particles> (new Particles(map_ptr));
    _particles_ptr->set_raysigma(_ray_sigma);
    _particles_ptr->set_size(_set_size);

}

GPF::~GPF() {}

void GPF::scan_callback(const sensor_msgs::LaserScan &msg) {
    // convert laser scan to point cloud

    if(!_listener.waitForTransform(
        msg.header.frame_id,
        _robot_frame,
        msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
        ros::Duration(0.1))) {
        ROS_WARN("GPF: scan transform is not found, time out.");
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    _projector.transformLaserScanToPointCloud(_robot_frame, msg, cloud, _listener, _cloud_range);

    // call cloud_callback function
    cloud_callback(cloud);
}
void GPF::cloud_callback(const sensor_msgs::PointCloud2 &msg) {
    _laser_time = msg.header.stamp;
    //ROS_INFO("GPF:cloud callback.");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>  cloud_temp;
    pcl::fromROSMsg(msg, cloud_temp);
    if(!_listener.waitForTransform(
		msg.header.frame_id,
		_robot_frame,
		msg.header.stamp,
		ros::Duration(0.1))) {
        ROS_WARN("GPF: cloud transform is not found, time out.");
        return;
    }

    pcl_ros::transformPointCloud(_robot_frame, cloud_temp, *cloud_ptr, _listener);
    _cloud_ptr = cloud_ptr;

    //ROS_INFO("GPF: before downsample");
    downsample();

    //structure_resample();

    // transform to imu frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*_cloud_ptr, *transformed_cloud_ptr, _imu_to_laser_transform);
    _cloud_ptr = transformed_cloud_ptr;

    // request prior from eskf
    _eskf_ptr->get_mean_pose(_mean_prior);
    _eskf_ptr->get_cov_pose(_cov_prior);
    
    
    // draw particles and propagate
    _particles_ptr->set_mean(_mean_prior);
    _particles_ptr->set_cov(_cov_prior);
    _particles_ptr->set_cloud(_cloud_ptr);
    _particles_ptr->propagate(_mean_sample, _cov_sample,
                              _mean_posterior, _cov_posterior);

    // update meas in eskf
    recover_meas();
    
    // check if the recovered pseudo mesure is valid
    for(int i=0; i<_mean_meas.size(); i++) {
        if(isnan(_mean_meas(i))) return;
    }
    for(int i=0; i<_cov_meas.size(); i++) {
        if(isnan(_cov_meas(i))) return;
    }
    // update eskf
    _eskf_ptr->update_meas_mean(_mean_meas);
    _eskf_ptr->update_meas_cov(_cov_meas);
    _eskf_ptr->update_meas_flag();


    /* TESTING PARTICLE FILTER RESUTLS*/
//    std::cout<< "mean sample:\n" << _mean_sample.transpose()<<std::endl;
//    std::cout<< "mean poster:\n" << _mean_posterior.transpose()<<std::endl;
//    std::cout<< "mean meas:\n" << _mean_meas.transpose()<<std::endl;
//    std::cout<< "cov prior:\n" << _cov_prior.diagonal().transpose()<<std::endl;
//    std::cout<< "cov poster:\n" << _cov_posterior.diagonal().transpose()<<std::endl;
//    std::cout<< "cov meas:\n" << _cov_meas.diagonal().transpose()<<std::endl;

    // publish needed resutls
    publish_pset();
    publish_cloud();
    //publish_posterior();
    publish_path();
//    publish_meas();
    publish_tf();
    //publish_pose();

}

void GPF::downsample() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr unif_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr xlim_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ylim_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr zlim_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);*/

    // down sampling
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(_cloud_ptr);
    uniform_sampling.setRadiusSearch(_cloud_resol);
    uniform_sampling.compute(sampled_indices);
    pcl::copyPointCloud (*_cloud_ptr, sampled_indices.points, *unif_cloud);

    // truncating in range
    /*pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(unif_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (-_cloud_range, _cloud_range);
    pass.filter(*xlim_cloud);

    pass.setInputCloud(xlim_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits (-_cloud_range, _cloud_range);
    pass.filter(*ylim_cloud);

    pass.setInputCloud(ylim_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (-_cloud_range, _cloud_range);
    pass.filter(*zlim_cloud);

    _cloud_ptr.reset();
    _cloud_ptr = zlim_cloud;

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
    condRem.setInputCloud(_cloud_ptr);
    condRem.setKeepOrganized(true);
    condRem.filter (*_cloud_ptr);*/

    _cloud_ptr->clear();
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = unif_cloud->begin();
		                                 it != unif_cloud->end();
						 it ++) {
	double dist = sqrt(it->x * it->x + it->y * it->y + it->z * it->z);

	if (dist > 0.9 && dist < _cloud_range) {
	    _cloud_ptr->push_back(*it);
	}
    }

    ROS_INFO_STREAM_THROTTLE(1.0, "GPF: Cloud size " << int(_cloud_ptr->size()));
}

void GPF::structure_resample() {
    if(!_cloud_ptr) {
        ROS_WARN("GPF: cloud ptr is null");
        return;
    }
    // Remove points too close or too far
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = _cloud_ptr->begin(); it != _cloud_ptr->end(); it++) {
        pcl::PointXYZ p = *it;
        double d = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if(d <= _cloud_range && d >= 0.8) {
            cloud.push_back(p);
        }
    }


    // Compute score for all points
    std::vector<double> score;
    score.resize(_cloud_ptr->size(), 0.0);
    int hfwin_size = 5;
    for(int i=0; i<cloud.size(); i++) {
        if(i-hfwin_size < 0 || i+hfwin_size >= cloud.size()) {
            score[i] = 0.5;
        } else {
            /*pcl::CentroidPoint<pcl::PointXYZ> lf_c, rg_c;
            for(int k=1; k <= hfwin_size; k++) {
                lf_c.add(cloud[i-k]);
                rg_c.add(cloud[i+k]);
            }
            pcl::PointXYZ lf_m, rg_m;
            lf_c.get(lf_m);
            rg_c.get(rg_m);*/
            pcl::PointXYZ lf_c, rg_c;
	    lf_c.x = 0.0; lf_c.y = 0.0; lf_c.z = 0.0; 
	    rg_c.x = 0.0; rg_c.y = 0.0; rg_c.z = 0.0;
	    for(int k=1; k<= hfwin_size; k++) {
	        lf_c = add(lf_c, cloud[i-k]);
		rg_c = add(rg_c, cloud[i+k]);
	    }
	    pcl::PointXYZ lf_m = multiply(lf_c, 1.0/hfwin_size);
	    pcl::PointXYZ rg_m = multiply(rg_c, 1.0/hfwin_size);
            pcl::PointXYZ lf_v, rg_v;
            lf_v = minus(lf_m, cloud[i]);
            rg_v = minus(rg_m, cloud[i]);
            double scr = norm(cross(lf_v, rg_v)) / (norm(lf_v) * norm(rg_v));
            score[i] = scr;

        }
    }

    // Sort the computed score
    std::vector<size_t> idx = sort_index(score);
    //ROS_INFO("GPF: sorted score.");

    // Looking for 100 feature points, 400 plane points
    pcl::PointCloud<pcl::PointXYZ> edge_cloud;
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    for(int i=0; i<cloud.size(); i++) {
        if(score[idx[i]] > 0.2) {
            edge_cloud.push_back(cloud[idx[i]]);
        }
        if(edge_cloud.size() >= 600) {
            break;
        }
    }
    for(int i=0; i<cloud.size(); i++) {
        if(score[idx[i]] < 0.2) {
            plane_cloud.push_back(cloud[idx[i]]);
        }
        if(plane_cloud.size() >= 200) {
            break;
        }
    }
    cloud = edge_cloud + plane_cloud;
    _cloud_ptr = cloud.makeShared();

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = _robot_frame;
    msg.header.stamp = _laser_time;

    _cloud_pub.publish(msg);
}
void GPF::recover_meas() {
    Eigen::Matrix<double, 6, 6> K;

    _cov_meas = (_cov_posterior.inverse() - _cov_sample.inverse()).inverse();
    check_posdef(_cov_meas);
    K = _cov_sample * ( _cov_sample + _cov_meas).inverse();
    _mean_meas = K.inverse() * (_mean_posterior - _mean_sample) + _mean_sample;

//    std::cout << "K:" << K << std::endl;

}

void GPF::check_posdef(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &R) {
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> rot;
    Eigen::Matrix<double, STATE_SIZE, 1> scl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> >
       eigenSolver(R);
    rot = eigenSolver.eigenvectors();
    scl = eigenSolver.eigenvalues();

    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> E;
    E.setZero();
    for (int ii=0; ii<STATE_SIZE; ++ii) {
        if(scl(ii,0)>0.0) {
            E(ii,ii) = scl(ii,0);
        }
        else {
            E(ii,ii) = 100.0;
        }
    }
    R = rot * E * rot.inverse();
}

void GPF::publish_meas() {
    nav_msgs::Odometry msg;

    msg.header.stamp = _laser_time;
    msg.header.frame_id = "world";

    msg.pose.pose.position.x = _mean_meas[0];
    msg.pose.pose.position.y = _mean_meas[1];
    msg.pose.pose.position.z = _mean_meas[2];

    tf::Quaternion q;
    q.setRPY(_mean_meas[3], _mean_meas[4], _mean_meas[5]);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            msg.pose.covariance[6*i+j] = _cov_meas(i, j);
        }
    }

    _meas_pub.publish(msg);
}

void GPF::publish_pset() {

    std::vector<std::vector<double> > c;
    c = compute_color(*_particles_ptr);
    std::vector<Particle> pset = _particles_ptr->get_pset();

    visualization_msgs::MarkerArray msg;

    for(int i=0; i<_set_size; i++) {

        tf::Vector3 position;
        tf::Quaternion quaternion;

        position.setValue(pset[i].state[0], pset[i].state[1], pset[i].state[2]);
        quaternion.setRPY(pset[i].state[3], pset[i].state[4], pset[i].state[5]);

        visualization_msgs::Marker m;
        m.header.frame_id = "world";
        m.header.stamp = _laser_time;
        m.ns = "particle_set";
        m.id = i;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = position.x();
        m.pose.position.y = position.y();
        m.pose.position.z = position.z();
        m.pose.orientation.x = quaternion.x();
        m.pose.orientation.y = quaternion.y();
        m.pose.orientation.z = quaternion.z();
        m.pose.orientation.w = quaternion.w();
        m.scale.x = 0.1;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.color.a = 1.0;
        m.color.r = c[i][0];
        m.color.g = c[i][1];
        m.color.b = c[i][2];

        msg.markers.push_back(m);
    }
   _pset_pub.publish(msg);
}

std::vector< std::vector<double> > GPF::compute_color(Particles pSet) {

    std::vector<Particle> particle = pSet.get_pset();
    std::vector< std::vector<double> > color;

    // find maximum and minimum weight
    double minWeight = 999.9;
    double maxWeight = -999.9;

    for(int i=0; i<_set_size; i++) {
        Particle p = particle[i];
        if(p.weight > maxWeight) maxWeight = p.weight;
        if(p.weight < minWeight) minWeight = p.weight;
    }
    double midWeight = (maxWeight + minWeight)/2.0;

    // compute color for each particle
    for(int i=0; i<particle.size(); i++) {

        std::vector<double> cRGB;
        Particle p = particle[i];
        if(minWeight <= p.weight && p.weight < midWeight) {
            cRGB.push_back((p.weight - minWeight)/(midWeight - minWeight));
            cRGB.push_back(1.0);
            cRGB.push_back(0.0);
        }
        else if(midWeight <= p.weight && p.weight <= maxWeight) {
            cRGB.push_back(1.0);
            cRGB.push_back(1.0 - (p.weight - midWeight)/(maxWeight - midWeight));
            cRGB.push_back(0.0);
        }
        else {
            cRGB.push_back(0.0);
            cRGB.push_back(0.0);
            cRGB.push_back(1.0);

        }
        color.push_back(cRGB);
    }
    return color;
}

void GPF::publish_cloud() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Matrix4d transform;
    //Eigen::Matrix3d rotation;
    //Eigen::Vector3d translation;

    //translation << _mean_prior[0], _mean_prior[1], _mean_prior[2];

    //rotation = Eigen::AngleAxisd(_mean_prior[3], Eigen::Vector3d(1.0,0.0,0.0))
    //         * Eigen::AngleAxisd(_mean_prior[4], Eigen::Vector3d(0.0,1.0,0.0))
    //         * Eigen::AngleAxisd(_mean_prior[5], Eigen::Vector3d(0.0,0.0,1.0));

    tf::Matrix3x3 rotation;
    rotation.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5]);
   
    transform << rotation[0][0],rotation[0][1],rotation[0][2], _mean_prior[0],
                 rotation[1][0],rotation[1][1],rotation[1][2], _mean_prior[1],
                 rotation[2][0],rotation[2][1],rotation[2][2], _mean_prior[2],
                 0, 0, 0, 1;

    pcl::transformPointCloud(*_cloud_ptr, cloud, transform);

    // stack into reconstructed map
    //*_recmap_ptr += cloud;
//    ROS_INFO("gpf: reconstructed map size %d\n", int(_recmap_ptr->size()));

    // publish scan
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = "world";
    msg.header.stamp = _laser_time;

    _cloud_pub.publish(msg);
}

void GPF::publish_posterior() {
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = _laser_time;
    msg.pose.pose.position.x = _mean_posterior[0];
    msg.pose.pose.position.y = _mean_posterior[1];
    msg.pose.pose.position.z = _mean_posterior[2];
    tf::Quaternion q;
    q.setRPY(_mean_posterior[3], _mean_posterior[4], _mean_posterior[5]);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    for(int i=0; i<STATE_SIZE; i++) {
        for(int j=0; j<STATE_SIZE; j++) {
            msg.pose.covariance[6*i+j] = _cov_posterior(i,j);
        }
    }
    _post_pub.publish(msg);
}

void GPF::publish_path() {
    geometry_msgs::PoseStamped msg;

    msg.header.frame_id = "world";
    msg.header.stamp = _laser_time;

    msg.pose.position.x = _mean_prior[0];
    msg.pose.position.y = _mean_prior[1];
    msg.pose.position.z = _mean_prior[2];

    tf::Quaternion q;
    q.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5]);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    
    _pose_deque.push_back(msg);
    if(_pose_deque.size() > 200) {
	    _pose_deque.pop_front();
    }

    _path.poses.clear();
    _path.header.frame_id = "world";
    _path.header.stamp = _laser_time;
    for(int i=0; i<_pose_deque.size(); i++) {
    	_path.poses.push_back(_pose_deque[i]);
    }

    _path_pub.publish(_path);
}

void GPF::publish_tf() {
    //ROS_INFO("GPF: Publishing tf.");
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(_mean_prior[0], _mean_prior[1], _mean_prior[2]));
    tf::Quaternion q;
    q.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5]);
    transform.setRotation(q);
    //tf::Transform body_to_world = transform.inverse();
    //_tf_br.sendTransform(tf::StampedTransform(body_to_world, _laser_time, _robot_frame, "world"));
    _tf_br.sendTransform(tf::StampedTransform(transform, _laser_time, "world", "robot"));
}

void GPF::publish_pose() {
    geometry_msgs::PoseStamped msg;

    msg.header.frame_id = "world";
    msg.header.stamp = _laser_time;

    msg.pose.position.x = _mean_prior[0];
    msg.pose.position.y = _mean_prior[1];
    msg.pose.position.z = _mean_prior[2];

    tf::Quaternion q;
    q.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5] + M_PI);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();

    _pose_pub.publish(msg);
}
