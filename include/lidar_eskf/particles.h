#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <Eigen/Dense>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include "lidar_eskf/EigenMultivariateNormal.hpp"
#include "lidar_eskf/map.h"

#define SET_SIZE 500
#define STATE_SIZE 6

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
    Particles(boost::shared_ptr<DistMap> map_ptr);
    ~Particles() {}

    void set_raysigma(double raysigma);
    void set_mean(Eigen::Matrix<double, STATE_SIZE, 1> &mean);
    void set_cov(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov);
    void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
    void draw_set();
    void weight_set();

    void get_posterior();

    void reproject_cloud(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud);
    void weight_particle(Particle &p, pcl::PointCloud<pcl::PointXYZ> &cloud);

    void propagate(Eigen::Matrix<double, STATE_SIZE, 1> &mean, Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &cov);

    std::vector<Particle> get_pset();
    std::vector<Particle> get_d_pset();

private:
    std::vector<Particle> _pset;
    std::vector<Particle> _d_pset;

    Eigen::Matrix<double, STATE_SIZE, 1> _mean_prior;
    Eigen::Matrix<double, STATE_SIZE, 1> _mean_posterior;

    Eigen::Matrix<double, STATE_SIZE, 1> _d_mean_prior;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _d_cov_prior;
    Eigen::Matrix<double, STATE_SIZE, 1> _d_mean_posterior;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _d_cov_posterior;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
    boost::shared_ptr<DistMap> _map_ptr;

    double _ray_sigma;

};
#endif // PARTICLES_H
