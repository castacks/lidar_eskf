#ifndef __EIGENMULTIVARIATENORMAL_HPP
#define __EIGENMULTIVARIATENORMAL_HPP

#include <Eigen/Dense>
#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

template<typename _Scalar, int _size>
class EigenMultivariateNormal
{
    boost::mt19937 rng;    // The uniform pseudo-random algorithm
    boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
       randN; // The 0-mean unit-variance normal generator

    Eigen::Matrix<_Scalar,_size,_size> rot;
    Eigen::Matrix<_Scalar,_size,1> scl;

    Eigen::Matrix<_Scalar,_size,1> mean;

public:
    EigenMultivariateNormal(const Eigen::Matrix<_Scalar,_size,1>& meanVec,
        const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
        : randN(rng,norm)
    {
        setCovar(covarMat);
        setMean(meanVec);
    }

    void setCovar(const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<_Scalar,_size,_size> >
           eigenSolver(covarMat);
        rot = eigenSolver.eigenvectors();
        scl = eigenSolver.eigenvalues();
        for (int ii=0;ii<_size;++ii) {
            scl(ii,0) = sqrt(scl(ii,0));
        }
    }

    void setMean(const Eigen::Matrix<_Scalar,_size,1>& meanVec)
    {
        mean = meanVec;
    }

    void nextSample(Eigen::Matrix<_Scalar,_size,1>& sampleVec)
    {
        for (int ii=0;ii<_size;++ii) {
            sampleVec(ii,0) = randN()*scl(ii,0);
        }
        sampleVec = rot*sampleVec + mean;
    }
    
};

#endif
