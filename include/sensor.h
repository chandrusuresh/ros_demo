#ifndef SENSOR_H
#define SENSOR_H

#include <Eigen/Dense>
#include "sensor_noise.h"

using namespace Eigen;
using namespace ros_demo;

namespace ros_demo
{
    class Sensor
    {
        public:
        Sensor(VectorXd mu_w,VectorXd std_w,VectorXd mu_rw,VectorXd std_rw)
        {
            size = mu_w.size();
            noise_mdl = NoiseModel(mu_w,std_w,mu_rw,std_rw);
        }
        VectorXd getOutput(VectorXd ground_truth,double dt)
        {
            VectorXd noise = noise_mdl.sample(dt);
            return ground_truth + noise.head(size) + noise.tail(size);
        }
        ~Sensor(){}
        private:
        NoiseModel noise_mdl;
        int size;
    };
}

#endif // SENSOR_H