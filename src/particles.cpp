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

void Particles::set_mean(Eigen::Matrix<double, STATE_SIZE, 1> &mean) {
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
        mvn.nextSample(_d_pset[i].state);

        _d_pset[i].weight = log(1.0/_set_size);
        _pset[i].weight = _d_pset[i].weight;

        // recover nominal states
        _pset[i].state = _mean_prior + _d_pset[i].state;

        // recover nominal states: rotation
        tf::Matrix3x3 rotation, d_rotation;
        rotation.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5]);
        d_rotation.setRPY(_d_pset[i].state[3], _d_pset[i].state[4], _d_pset[i].state[5]);
        rotation  = rotation * d_rotation;
        rotation.getRPY(_pset[i].state[3], _pset[i].state[4], _pset[i].state[5]);
    }

    _d_mean_sample.setZero();
    _d_cov_sample.setZero();

    for(int i=0; i<_set_size; i++) {
        _d_mean_sample += _d_pset[i].state / _set_size;
    }
    for(int i=0; i<_set_size; i++) {
        _d_cov_sample += (_d_pset[i].state - _d_mean_sample)*(_d_pset[i].state - _d_mean_sample).transpose() / _set_size;
    }
}

void Particles::weight_set() {
#pragma omp parallel for
    for(int i=0; i<_set_size; i++) {
        // reproject cloud on to each particle
        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        reproject_cloud(_pset[i], cloud_transformed);

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

    // transform cloud
    Eigen::Matrix4d transform;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    translation << p.state[0],
                   p.state[1],
                   p.state[2];

    rotation = Eigen::AngleAxisd(p.state[3], Eigen::Vector3d(1.0,0.0,0.0))
             * Eigen::AngleAxisd(p.state[4], Eigen::Vector3d(0.0,1.0,0.0))
             * Eigen::AngleAxisd(p.state[5], Eigen::Vector3d(0.0,0.0,1.0));
    transform << rotation, translation,
                 0, 0, 0, 1;

    pcl::transformPointCloud(*_cloud_ptr, cloud, transform);
}

void Particles::weight_particle(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    for(int i=0; i<cloud.size(); i++) {
        // the end point of one ray
        octomap::point3d end_pnt(cloud[i].x, cloud[i].y, cloud[i].z);

        // look up the distance to nearest obstacle
        double dist = _map_ptr->get_dist(end_pnt);

        // find weight through normal distribution
        char grid_flag = _map_ptr->get_gridmask(end_pnt);
        if(grid_flag != 2) {
            if(dist >= 0.0 && dist <= 2.0*_ray_sigma) {
                p.weight += log_likelihood(dist, _ray_sigma);
            } else {
                p.weight += log_likelihood(2.0*_ray_sigma, _ray_sigma);
            }
        } else {
            if(dist >= 0.0 && dist <= 0.5*_ray_sigma) {
                p.weight += log_likelihood(dist, _ray_sigma);
            } else {
                p.weight += log_likelihood(0.5*_ray_sigma, _ray_sigma);
            }
        }
    }
}

void Particles::get_posterior() {

    _d_mean_posterior.setZero();
    _d_cov_posterior.setZero();

    for(int i=0; i<_set_size; i++) {
        _d_mean_posterior += exp(_d_pset[i].weight) * _d_pset[i].state;
    }
    for(int i=0; i<_set_size; i++) {
        _d_cov_posterior += exp(_d_pset[i].weight) * (_d_pset[i].state - _d_mean_posterior) * (_d_pset[i].state - _d_mean_posterior).transpose();
    }
}

void Particles::propagate(Eigen::Matrix<double, STATE_SIZE, 1> &mean_prior,
                          Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov_prior,
                          Eigen::Matrix<double, STATE_SIZE, 1> &mean_posterior,
                          Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov_posterior) {

    // generate particles
    draw_set();

    // weight each particles
    weight_set();

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
