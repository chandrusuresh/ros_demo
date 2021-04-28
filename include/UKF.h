#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <sensor_msgs/Imu.h>

namespace ros_demo
{
    class UKF
    {
        public:
        UKF();
        ~UKF();
        // virtual ~UKF();
        
        ///* initially set to false, set to true in first call of ProcessMeasurement
        bool is_initialized_;

        ////////////////////////
        ///////// UKF //////////
        ////////////////////////

        ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        VectorXd x_;

        ///* state covariance matrix
        MatrixXd P_;

        ///* predicted sigma points matrix
        MatrixXd Xsig_pred_;

        ///* Weights of sigma points
        VectorXd weights_;

        ///* State dimension
        int n_x_;

        ///* Augmented state dimension
        int n_aug_;

        ///* Sigma point spreading parameter
        double lambda_;

        ///* Number of sigma points
        int n_sig_;

        /////////////////////////////////////
        ///////// Noise Parameters //////////
        /////////////////////////////////////

        // Process noise standard deviation for position in m
        double std_s_ = 0.1;

        // Process noise standard deviation for velocity in m/s
        double std_v_ = 0.1;

        // Process noise standard deviation for orientation in rad
        double std_phi_ = 0.02;

        // Measurement noise for longitudinal acceleration
        double std_a_meas_ = 0.1;

        // Measurement noise for angular velocity
        double std_w_meas_ = 0.1;

        ////////////////////////////////////
        ///////// Misc Parameters //////////
        ////////////////////////////////////

        ///* time when the state is true, in us
        long long time_us_;                  
        long long previous_timestamp_;


        void ProcessMeasurement(sensor_msgs::ImuConstPtr& imu_meas);
        void Prediction(double delta_t);
        void UpdateIMU(sensor_msgs::ImuConstPtr& imu_meas);
        MatrixXd GenerateSigmaPoints();
        VectorXd SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug);
        VectorXd VecToQuaternionMath(VectorXd angle);
        Quaterniond VecToQuaternion(VectorXd angle);
        VectorXd MultiplyQuaternions(VectorXd p, VectorXd q);
        void QuaternionAverage(MatrixXd q_sig,VectorXd q_prev);
        VectorXd InverseQuaternion(VectorXd q);
        VectorXd ExpQuaternion(VectorXd q);
        private:
        VectorXd gravity_W_ = {0,0,9.81};
        VectorXd q_mean;
        MatrixXd q_err;
    };
}

#endif // UKF_H