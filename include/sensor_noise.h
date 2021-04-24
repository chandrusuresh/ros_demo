#ifndef SENSOR_NOISE_H
#define SENSOR_NOISE_H

#include <Eigen/Dense>
#include "Utils.h"

namespace ros_demo
{
    class NoiseModel
    {
        public:
        
        NoiseModel(VectorXd mu_w,VectorXd std_w,VectorXd mu_rw,VectorXd std_rw)
        {
            VectorXd mu  << mu_w,mu_rw;
            
            VectorXd std_full << std_w,std_rw;
            MatrixXd cov << std_full.asDiagonal()*std_full.asDiagonal();

            noise = new Gaussian(mu.size());
        }
        ~NoiseModel();
        VectorXd sample(double dt)
        {
            assert(dt > 0);
            VectorXd delT << 1/dt,1/dt,1/dt,dt,dt,dt;
            MatrixXd new_cov = delT.asDiagonal()*cov;

            noise.setMean(mu);
            noise.setCovariance(new_cov);
            noise.setSampler();
            VectorXd noise = noise.sample(1).col(0);

            // Reset mean of random walk to current sample value
            int rw_size = mu.size()/2;
            mu.tail<rw_size> = noise.tail<rw_size>;
            return noise;
        }
        private:
        VectorXd mu;
        MatrixXd cov;
        Gaussian noise;
    }
};

#endif // SENSOR_NOISE_H