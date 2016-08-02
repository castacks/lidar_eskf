#include "lidar_eskf/gpf.h"

GPF::GPF(ros::NodeHandle &nh, DistMap &map) {
    _position_prior.setZero();
    _theta_prior.setZero();
    _rotation_prior.setIdentity();
    _quaternion_prior.setRPY(0.0,0.0,0.0);

    _mean_prior.setZero();
    _mean_posterior.setZero();
    _mean_meas.setZero();

    _cov_prior.setZero();
    _cov_posterior.setZero();
    _cov_meas.setZero();

    _cloud_sub = nh.subscribe("cloud", 100, &GPF::cloud_callback, this);
    _odom_sub = nh.subscribe("odom", 100, &GPF::odom_callback, this);
    _meas_pub = nh.advertise<nav_msgs::Odometry>("meas", 100);

    _map_ptr.reset(&map);

}

void GPF::odom_callback(const nav_msgs::Odometry &msg) {

    _position_prior.setX(msg.pose.pose.position.x);
    _position_prior.setY(msg.pose.pose.position.y);
    _position_prior.setZ(msg.pose.pose.position.z);

    _quaternion_prior.setX(msg.pose.pose.orientation.x);
    _quaternion_prior.setY(msg.pose.pose.orientation.y);
    _quaternion_prior.setZ(msg.pose.pose.orientation.z);
    _quaternion_prior.setW(msg.pose.pose.orientation.w);

    _rotation_prior.setRotation(_quaternion_prior);

    double r, p, y;
    _rotation_prior.getRPY(r, p, y);

    _theta_prior.setX(r);
    _theta_prior.setX(p);
    _theta_prior.setX(y);

    _mean_prior[0] = _position_prior.x();
    _mean_prior[1] = _position_prior.y();
    _mean_prior[2] = _position_prior.z();

    _mean_prior[3] = _theta_prior.x();
    _mean_prior[3] = _theta_prior.x();
    _mean_prior[3] = _theta_prior.x();

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            _cov_prior(i,j) = msg.pose.covariance[i*6+j];
        }
    }
}

void GPF::cloud_callback(const sensor_msgs::PointCloud2 &msg) {
    _laser_time = msg.header.stamp;

    pcl::fromROSMsg(msg, *_cloud_ptr);
    downsample();

    double ray_sigma, log_offset;
    Particles pset(_mean_prior, _cov_prior, *_cloud_ptr, *_map_ptr, ray_sigma, log_offset);
    pset.propagate(_mean_posterior, _cov_posterior);
}

void GPF::downsample() {
    // down sampling
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(_cloud_ptr);
    uniform_sampling.setRadiusSearch(0.1);
    uniform_sampling.compute(sampled_indices);
    pcl::copyPointCloud (*_cloud_ptr, sampled_indices.points, *_cloud_ptr);

    // truncating in range
    double rangeLim = 10.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*_cloud_ptr);

    pass.setInputCloud(_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*_cloud_ptr);

    pass.setInputCloud(_cloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*_cloud_ptr);

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
    condRem.filter (*_cloud_ptr);
}

void GPF::recover_pseudo_meas() {
    Eigen::Matrix<double, 6, 6> C;
    Eigen::Matrix<double, 6, 6> K;
    Eigen::Matrix<double, 6, 6> H = Eigen::MatrixXd::Identity(6, 6);

    C = (_cov_posterior.inverse() - _cov_prior.inverse()).inverse();
    check_posdef(C);

    K = _cov_prior.inverse() * H.transpose() * (H * _cov_prior * H.transpose() + C).inverse();
    _mean_meas = K.inverse() * (_mean_posterior - _mean_prior) + _mean_prior;
    _cov_meas = C;
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
