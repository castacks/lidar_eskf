#include "lidar_eskf/particles.h"



Particles::Particles(Eigen::Matrix<double, STATE_SIZE, 1> &mean,
                     Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov,
                     pcl::PointCloud<pcl::PointXYZ> &cloud) {
    _mean_prior.setZero();
    _mean_posterior.setZero();
    _d_mean_prior.setZero();
    _d_mean_posterior.setZero();
    _d_cov_prior.setZero();
    _d_cov_posterior.setZero();

    _mean_prior = mean;
    _d_cov_prior = cov;

    _cloud.resize(cloud.size());
    _cloud = cloud;

    _pset.resize(SET_SIZE);
    _d_pset.resize(SET_SIZE);
}

void Particles::init_set() {

    EigenMultivariateNormal<double, STATE_SIZE> mvn (_d_mean_prior, _d_cov_prior);

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
        // transform cloud
        tf::Matrix3x3 R;
        tf::Vector3 t;
        R.setRPY(_pset[i].state[3], _pset[i].state[4], _pset[i].state[5]);
        t.setValue(_pset[i].state[6], _pset[i].state[7], _pset[i].state[8]);

        Eigen::Matrix<double,4,4> transform;
        Eigen::Matrix<double,3,3> rotation;
        Eigen::Matrix<double,3,1> translation;
        tf::matrixTFToEigen(R, rotation);
        tf::vectorTFToEigen(t, translation);
        transform << rotation, translation,
                     0, 0, 0, 1;

        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        pcl::transformPointCloud(_cloud, cloud_transformed, transform);

        // weight particle

        // compute weight sum
    }

    // normalize weight
}
