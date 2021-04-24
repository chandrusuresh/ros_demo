#ifndef SENSOR_NOISE_H
#define SENSOR_NOISE_H

#include <Eigen/Dense>
// #include "Utils.h"
#include "eigenmvn.h"

using namespace Eigen;
namespace ros_demo
{
    class NoiseModel
    {
        public:
        NoiseModel(){}
        NoiseModel(VectorXd mu_w,VectorXd std_w,VectorXd mu_rw,VectorXd std_rw)
        {
            mu << mu_w,mu_rw;
            
            VectorXd std_full(mu.size());
            std_full << std_w,std_rw;

            MatrixXd std_mat = std_full.asDiagonal();
            cov = std_mat*std_mat;
        }
        ~NoiseModel(){}
        VectorXd sample(double dt)
        {
            assert(dt > 0);
            VectorXd delT(mu.size());
            delT << 1/dt,1/dt,1/dt,dt,dt,dt;
            MatrixXd new_cov = delT.asDiagonal()*cov;

            EigenMultivariateNormal<double> sampler(mu,new_cov);
            VectorXd noise = sampler.samples(1).col(0);

            // Reset mean of random walk to current sample value
            int rw_size = mu.size()/2;
            mu.tail(rw_size) = noise.tail(rw_size);
            return noise;
        }
        private:
        VectorXd mu = VectorXd(6);
        MatrixXd cov = MatrixXd(6,6);
    };
}

#endif // SENSOR_NOISE_H