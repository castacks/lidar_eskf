#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <Eigen/Dense>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include "lidar_eskf/EigenMultivariateNormal.hpp"

#define SET_SIZE 500
#define STATE_SIZE 15

struct Particle {
    Eigen::Matrix<double, STATE_SIZE, 1> state;
    double weight;
    Particle() {
        state.setZero();
        weight = 0.0;
    }
};

class Particles {
public:
    Particles(Eigen::Matrix<double, STATE_SIZE, 1> &mean,
              Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov,
              pcl::PointCloud<pcl::PointXYZ> &cloud);
    ~Particles() {}

    void init_map();

    void init_set();
    void weight_set();

    void get_posterior(Eigen::Matrix<double, STATE_SIZE, 1> &mean, Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov);
    void get_pseudo_meas();

private:
    std::vector<Particle> _pset;
    std::vector<Particle> _d_pset;

    Eigen::Matrix<double, STATE_SIZE, 1> _mean_prior;
    Eigen::Matrix<double, STATE_SIZE, 1> _mean_posterior;

    Eigen::Matrix<double, STATE_SIZE, 1> _d_mean_prior;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _d_cov_prior;
    Eigen::Matrix<double, STATE_SIZE, 1> _d_mean_posterior;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _d_cov_posterior;

    pcl::PointCloud<pcl::PointXYZ> _cloud;

};
#endif // PARTICLES_H
