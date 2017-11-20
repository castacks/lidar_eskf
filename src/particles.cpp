/*
* Copyright (c) 2016 Carnegie Mellon University, Weikun Zhen <weikunz@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "lidar_eskf/particles.h"

inline double log_likelihood(double x, double sigma) {
    return -0.91893853320467274178 - log(sigma) - 0.5 * x * x / (sigma * sigma);
}
static EigenMultivariateNormal<double, STATE_SIZE> mvn(Eigen::MatrixXd::Zero(STATE_SIZE,1),
                                                       Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE));

Particles::Particles(boost::shared_ptr<DistMap> map_ptr) : _map_ptr(map_ptr)
{
    _mean_prior.setZero();
    _mean_posterior.setZero();
    _d_mean_prior.setZero();
    _d_mean_sample.setZero();
    _d_mean_posterior.setZero();
    _d_cov_prior.setZero();
    _d_cov_sample.setZero();
    _d_cov_posterior.setZero();

}

void Particles::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    _cloud_ptr = cloud_ptr;
}

void Particles::set_mean(Eigen::Matrix<double, 7, 1> &mean) {
    _mean_prior = mean;
}

void Particles::set_cov(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov) {
    _d_cov_prior = cov;
}

void Particles::set_raysigma(double raysigma) {
    _ray_sigma = raysigma;
}

void Particles::set_size(int set_size) {
    _set_size = set_size;

    _pset.resize(_set_size);
    _d_pset.resize(_set_size);
}

void Particles::draw_set() {

    mvn.setMean(_d_mean_prior);
    mvn.setCovar(_d_cov_prior);

    for(int i=0; i<_set_size; i++) {
        // random sample error states
        Eigen::Matrix<double, 6, 1> twist;
        mvn.nextSample(twist);
        _d_pset[i].translation = twist.block<3,1>(0,0);
        _d_pset[i].angle_axis = twist.block<3,1>(3,0);

        _d_pset[i].weight = log(1.0/_set_size);
        _pset[i].weight = _d_pset[i].weight;

        // recover nominal states
        _pset[i].translation = _mean_prior.block<3,1>(0,0) + _d_pset[i].translation;

        // recover nominal states: rotation
        Eigen::Matrix3d d_rotation = angle_axis_to_rotation_matrix(_d_pset[i].angle_axis);
        _pset[i].rotation = Eigen::Quaterniond(_mean_prior[3],_mean_prior[4],
                _mean_prior[5],_mean_prior[6]) * Eigen::Quaterniond(d_rotation);
    }

    _d_mean_sample.setZero();
    _d_cov_sample.setZero();

    for(int i=0; i<_set_size; i++) {
        _d_mean_sample.block<3,1>(0,0) += _d_pset[i].translation / _set_size;
        _d_mean_sample.block<3,1>(3,0) += _d_pset[i].angle_axis / _set_size;
    }
    for(int i=0; i<_set_size; i++) {
        Eigen::Matrix<double, 6, 1> twist;
        twist << _d_pset[i].translation - _d_mean_sample.block<3,1>(0,0),
                 _d_pset[i].angle_axis - _d_mean_sample.block<3,1>(3,0);
        _d_cov_sample += twist*twist.transpose() / _set_size;
    }
}

void Particles::weight_set() {
//#pragma omp parallel for
    for(int i=0; i<_set_size; i++) {
        // reproject cloud on to each particle
        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
//        reproject_cloud(_pset[i], cloud_transformed);
        pcl::transformPointCloud(*_cloud_ptr, cloud_transformed, _pset[i].translation, _pset[i].rotation);
        // weight particle
        weight_particle(_pset[i], cloud_transformed);
    }

//    std::cout << "Particles: weight_1 = ";
//    for(int i=0; i<_set_size; i++) {
//        std::cout << exp(_pset[i].weight) << "\t";
//    }
//    std::cout << std::endl;

    // stabilize weights, offset weight values to [-200.0, 0.0] range
    double max_weight(-INFINITY);
    for(int i=0; i<_set_size; i++) {
        if(_pset[i].weight > max_weight) max_weight = _pset[i].weight;
    }
    for(int i=0; i<_set_size; i++) {
        _pset[i].weight -= max_weight;
        if(_pset[i].weight < -200.0) _pset[i].weight = -200.0;
    }

//    std::cout << "Particles: weight_2 = ";
//    for(int i=0; i<_set_size; i++) {
//        std::cout << exp(_pset[i].weight) << "\t";
//    }
//    std::cout << std::endl;

    // normalize weight
    double weight_sum = 0.0;
    for(int i=0; i<_set_size; i++) {
        weight_sum += exp(_pset[i].weight);
    }
    double log_weight_sum = log(weight_sum);
    for(int i=0; i<_set_size; i++) {
        _pset[i].weight -= log_weight_sum;
        _d_pset[i].weight = _pset[i].weight;
    }

//    std::cout << "Particles: weight_3 = ";
//    for(int i=0; i<_set_size; i++) {
//        std::cout << exp(_pset[i].weight) << "\t";
//    }
//    std::cout << std::endl;
}

void Particles::reproject_cloud(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud) {

    pcl::transformPointCloud(*_cloud_ptr, cloud, p.translation, p.rotation);
}

void Particles::weight_particle(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    std::vector<double> weight;
    weight.resize(cloud.size());

//#pragma omp parallel for
    for(int i=0; i<cloud.size(); i++) {
        // the end point of one ray
        octomap::point3d end_pnt(cloud[i].x, cloud[i].y, cloud[i].z);

        // look up the distance to nearest obstacle
        double dist = _map_ptr->get_dist(end_pnt);

        // find weight through normal distribution
        char grid_flag = _map_ptr->get_gridmask(end_pnt);
        if(grid_flag != 2) {
            if(dist >= 0.0 && dist <= 2.0*_ray_sigma) {
                weight[i] = log_likelihood(dist, _ray_sigma);
            } else {
                weight[i] = log_likelihood(2.0*_ray_sigma, _ray_sigma);
            }
        } else {
            if(dist >= 0.0 && dist <= 0.5*_ray_sigma) {
                weight[i] = log_likelihood(dist, _ray_sigma);
            } else {
                weight[i] = log_likelihood(0.5*_ray_sigma, _ray_sigma);
            }
        }
    }

    for(int i=0; i<cloud.size(); i++) {
        p.weight += weight[i];
    }
}

void Particles::get_posterior() {

    _d_mean_posterior.setZero();
    _d_cov_posterior.setZero();

    for(int i=0; i<_set_size; i++) {
        _d_mean_posterior.block<3,1>(0,0) += exp(_d_pset[i].weight) * _d_pset[i].translation;
        _d_mean_posterior.block<3,1>(3,0) += exp(_d_pset[i].weight) * _d_pset[i].angle_axis;

    }
    for(int i=0; i<_set_size; i++) {
        Eigen::Matrix<double, 6, 1> twist;
        twist.block<3,1>(0,0) = _d_pset[i].translation - _d_mean_posterior.block<3,1>(0,0);
        twist.block<3,1>(3,0) = _d_pset[i].angle_axis - _d_mean_posterior.block<3,1>(3,0);
        _d_cov_posterior += exp(_d_pset[i].weight) * (twist) * (twist).transpose();
    }
}

//void Particles::propagate(Eigen::Matrix<double, STATE_SIZE, 1> &mean_prior,
//                          Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov_prior,
//                          Eigen::Matrix<double, STATE_SIZE, 1> &mean_posterior,
//                          Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov_posterior) {

//    // generate particles
//    draw_set();

//    // weight each particles
//    double start = ros::Time::now().toSec();
//    weight_set();
//    //ROS_INFO("weighting time: %f",ros::Time::now().toSec() - start );

//    // compute weighted mean and cov
//    get_posterior();

//    mean_prior = _d_mean_sample;
//    cov_prior = _d_cov_sample;
//    mean_posterior = _d_mean_posterior;
//    cov_posterior  = _d_cov_posterior;
//}
void Particles::propagate(Eigen::Matrix<double, 6, 1> &mean_prior,
                          Eigen::Matrix<double, 6, 6> &cov_prior,
                          Eigen::Matrix<double, 6, 1> &mean_posterior,
                          Eigen::Matrix<double, 6, 6> &cov_posterior) {

    // generate particles
    draw_set();

    // weight each particles
    double start = ros::Time::now().toSec();
    weight_set();
    //ROS_INFO("weighting time: %f",ros::Time::now().toSec() - start );

    // compute weighted mean and cov
    get_posterior();

    mean_prior = _d_mean_sample;
    cov_prior = _d_cov_sample;
    mean_posterior = _d_mean_posterior;
    cov_posterior  = _d_cov_posterior;
}
std::vector<Particle> Particles::get_pset() {
    return _pset;
}

std::vector<Particle> Particles::get_d_pset() {
    return _d_pset;
}
