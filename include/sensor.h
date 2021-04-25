#ifndef SENSOR_H
#define SENSOR_H

#include <Eigen/Dense>
#include <vector>
#include "sensor_noise.h"

using namespace std;
using namespace Eigen;
using namespace ros_demo;

namespace ros_demo
{
    class Sensor
    {
        public:
        Sensor(){}
        Sensor(VectorXd tau,VectorXd std_wn,VectorXd std_rw,VectorXd std_tob)
        {
            noise_mdl.resize(tau.size());
            for (int i=0;i<tau.size();i++) noise_mdl[i] = NoiseModel(tau[i],std_wn[i],std_rw[i],std_tob[i]);
        }
        VectorXd getOutput(VectorXd ground_truth,double dt)
        {
            VectorXd noise(noise_mdl.size());
            for (int i=0;i<noise_mdl.size();i++) noise[i] = noise_mdl[i].sample(dt);
            return ground_truth + noise;
        }
        ~Sensor(){}
        private:
        vector<NoiseModel> noise_mdl;
    };

    class IMU
    {
        public:
        IMU(Sensor gyro,Sensor accmeter)
        {
            gyroscope = gyro;
            accelerometer = accmeter;
        }
        VectorXd getGyroscopeReading(VectorXd gnd_truth,double dt)
        {
            return gyroscope.getOutput(gnd_truth,dt);
        }
        VectorXd getAccelerometerReading(VectorXd gnd_truth,double dt)
        {
            return accelerometer.getOutput(gnd_truth,dt);
        }
        private:
        Sensor gyroscope;
        Sensor accelerometer;
    };
}

#endif // SENSOR_H