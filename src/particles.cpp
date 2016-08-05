#include "lidar_eskf/particles.h"

inline double log_likelihood(double x, double sigma) {
    return -0.91893853320467274178 - log(sigma) - 0.5 * pow(x/sigma, 2.0);
}

Particles::Particles(ros::NodeHandlePtr nh_ptr,
                     Eigen::Matrix<double, STATE_SIZE, 1> &mean,
                     Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
                     boost::shared_ptr<DistMap> map_ptr) : _cloud_ptr(cloud_ptr), _map_ptr(map_ptr)
{
    _mean_prior.setZero();
    _mean_posterior.setZero();
    _d_mean_prior.setZero();
    _d_mean_posterior.setZero();
    _d_cov_prior = Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE);
    _d_cov_posterior = Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE);

    _mean_prior = mean;
    _d_cov_prior = cov;

    _pset.resize(SET_SIZE);
    _d_pset.resize(SET_SIZE);

    nh_ptr->param("ray_sigma", _ray_sigma, 1.0);
    nh_ptr->param("log_offset", _log_offset, 300.0);
}

void Particles::init_set() {

    static EigenMultivariateNormal<double, STATE_SIZE> mvn(_d_mean_prior, _d_cov_prior);

    mvn.setMean(_d_mean_prior);
    mvn.setCovar(_d_cov_prior);

    for(int i=0; i<SET_SIZE; i++) {
        // random sample error states
        mvn.nextSample(_d_pset[i].state);
        _d_pset[i].weight = log(1.0/SET_SIZE);

        // recover nominal states
        _pset[i].state = _mean_prior + _d_pset[i].state;

        // recover nominal states: rotation
        tf::Matrix3x3 rotation, d_rotation;
        rotation.setRPY(_mean_prior[3], _mean_prior[4], _mean_prior[5]);
        d_rotation.setRPY(_d_pset[i].state[3], _d_pset[i].state[4], _d_pset[i].state[5]);
        rotation  = rotation * d_rotation;
        rotation.getRPY(_pset[i].state[3], _pset[i].state[4], _pset[i].state[5]);
    }
}

void Particles::weight_set() {
    for(int i=0; i<SET_SIZE; i++) {
        // reproject cloud on to each particle
        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        reproject_cloud(_pset[i], cloud_transformed);

        // weight particle
        weight_particle(_pset[i], cloud_transformed);
    }

    // normalize weight
    double weight_sum = 0.0;
    for(int i=0; i<SET_SIZE; i++) {
        weight_sum += exp(_pset[i].weight);
    }
    double log_weight_sum = log(weight_sum);
    for(int i=0; i<SET_SIZE; i++) {
        _pset[i].weight -= log_weight_sum;
    }

    // transfer weights to error state particles
    for(int i=0; i<SET_SIZE; i++) {
        _d_pset[i].weight = _pset[i].weight;
    }
}

void Particles::reproject_cloud(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud) {

    // transform cloud
    tf::Matrix3x3 R;
    tf::Vector3 t;
    R.setRPY(p.state[3], p.state[4], p.state[5]);
    t.setValue(p.state[6], p.state[7], p.state[8]);

    Eigen::Matrix<double,4,4> transform;
    Eigen::Matrix<double,3,3> rotation;
    Eigen::Matrix<double,3,1> translation;
    tf::matrixTFToEigen(R, rotation);
    tf::vectorTFToEigen(t, translation);
    transform << rotation, translation,
                 0, 0, 0, 1;

    pcl::transformPointCloud(*_cloud_ptr, cloud, transform);
}

void Particles::weight_particle(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    for(int i=0; i<cloud.size(); i++) {
        // the end point of one ray
        octomap::point3d end_pnt(cloud[i].x, cloud[i].y, cloud[i].z);

        // look up the distance to nearest obstacle
        double dist = _map_ptr->get_dist_map()->getDist(end_pnt);

        // lookup grid type
        char grid_flag = _map_ptr->get_dist_map()->getGridMask(end_pnt);

        // find weight through normal distribution
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

        // offset weight for numerical stabilization
        p.weight += _log_offset;

    }
}

void Particles::get_posterior() {

    for(int i=0; i<SET_SIZE; i++) {
        _d_mean_posterior += exp(_d_pset[i].weight) * _d_pset[i].state / SET_SIZE;
    }
    for(int i=0; i<SET_SIZE; i++) {
        _d_cov_posterior += (_d_pset[i].state - _d_mean_posterior) * (_d_pset[i].state - _d_mean_posterior).transpose();
    }
}

void Particles::propagate(Eigen::Matrix<double, STATE_SIZE, 1> &mean, Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov) {

    // generate particles
    init_set();

    // weight each particles
    weight_set();

    // compute weighted mean and cov
    get_posterior();

    mean = _d_mean_posterior;
    cov  = _d_cov_posterior;
}

std::vector<Particle> Particles::get_pset() {
    return _pset;
}

std::vector<Particle> Particles::get_d_pset() {
    return _d_pset;
}
