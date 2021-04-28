#include "UKF.h"
using namespace std;
using namespace Eigen;

namespace ros_demo
{
    UKF::UKF()
    {
        gravity_W_ = VectorXd(3);
        gravity_W_ << 0,0,9.81;

        // Dimension of state vector: s,v,q,w,acc
        n_x_ = 16;
        // 6 dummy variables in acc, w in x,y & z
        n_aug_ = 16;
        n_sig_ = 2*n_aug_ + 1;
        lambda_ = 3 - n_aug_;
        weights_ = VectorXd(n_sig_);
        double weight = lambda_/(lambda_+n_aug_);
        weights_(0) = weight;
        for (int i=1; i < n_sig_; i++)
        {
            weight = 0.5/(lambda_+n_aug_);
            weights_(i) = weight;
        }

        // Process noise standard deviation for position in m
        std_s_ = 0.1;

        // Process noise standard deviation for velocity in m/s
        std_v_ = 0.1;

        // Process noise standard deviation for orientation in rad
        std_phi_ = 0.02;

        // Measurement noise for longitudinal acceleration
        std_a_meas_ = 0.1;

        // Measurement noise for angular velocity
        std_w_meas_ = 0.1;

        // initial state vector
        x_ = VectorXd(n_x_);
        x_.fill(0.0);

        // initial covariance matrix
        P_ = MatrixXd(n_x_-1,n_x_-1);
        for (int i=0;i<3;i++) P_(i,i) = std_phi_*std_phi_;
        for (int i=3;i<6;i++) P_(i,i) = std_s_*std_s_;
        for (int i=6;i<9;i++) P_(i,i) = std_v_*std_v_;
        for (int i=9;i<12;i++) P_(i,i) = std_w_meas_*std_w_meas_;
        for (int i=12;i<n_x_-1;i++) P_(i,i) = std_a_meas_*std_a_meas_;

        // Sigma Point predictions
        Xsig_pred_ = MatrixXd(n_x_,n_sig_);

        // Initial q_mean and error
        q_mean = VectorXd(4);
        q_err  = MatrixXd(3,n_sig_);
    }
    UKF::~UKF(){}
    void UKF::Prediction(double delta_t)
    {
        double dt2 = delta_t*delta_t;

        //predict sigma points
        MatrixXd Xsig_aug = GenerateSigmaPoints();

        //write predicted sigma points into right column
        VectorXd x_next = SigmaPointPrediction(delta_t,Xsig_aug);

        MatrixXd P_next = MatrixXd(n_x_-1,n_x_-1);
        P_next.fill(0.0);
        for (int i=0; i<n_sig_;i++)
        {
            VectorXd del_x(x_next.size()-1);
            del_x.head(3) = q_err.col(i);
            del_x.tail(12) = Xsig_pred_.col(i).tail(12) - x_next.tail(12);            
            P_next += weights_(i)*del_x*del_x.transpose();
        }
        x_ = x_next;
        P_ = P_next;
    }
    void UKF::UpdateIMU(const sensor_msgs::ImuConstPtr& imu_meas)
    {
        int n_z = 6;
        VectorXd z = VectorXd(n_z);
        z << imu_meas->angular_velocity.x,imu_meas->angular_velocity.y,imu_meas->angular_velocity.z,
             imu_meas->linear_acceleration.x,imu_meas->linear_acceleration.y,imu_meas->linear_acceleration.z;
        
        MatrixXd Zsig = MatrixXd(n_z,n_sig_);
        VectorXd z_pred = VectorXd(n_z);
        z_pred.fill(0.0);
        for (int i=0; i < n_sig_; i++)
        {
            VectorXd x = Xsig_pred_.col(i);
            VectorXd z1(n_z);
            z1.head(3) = x.segment(10,12);
            
            Quaterniond q;
            q.w() = x(0);
            q.vec() = x.segment(1,3);
            Quaterniond qi = q.inverse();
            VectorXd gravity_comp = qi*gravity_W_;

            z1.tail(3) = x.tail(3)-gravity_comp;
            Zsig.col(i) = z1;
            z_pred += weights_(i)*z1;
        }
                    
        MatrixXd R = MatrixXd::Zero(n_z,n_z);
        int n_dum = n_sig_-n_x_;
        for (int i=0;i<n_dum/2;i++) R(i,i) = std_w_meas_*std_w_meas_;
        for (int i=n_dum/2;i<n_dum;i++) R(i,i) = std_a_meas_*std_a_meas_;
        
        MatrixXd Tc = MatrixXd::Zero(n_sig_,n_z);
        MatrixXd S = R;
        for (int i=0; i<n_sig_;i++)
        {
            VectorXd delta_z = Zsig.col(i)-z_pred;
            VectorXd delta_x(x_.size()-1);
            delta_x.head(3) = q_err.col(i);
            delta_x.tail(12) = Xsig_pred_.col(i).tail(12) - x_.tail(12);
            
            S += weights_(i)*delta_z*delta_z.transpose();
            Tc += weights_(i)*delta_x*delta_z.transpose();
        }
        MatrixXd K = Tc*S.inverse();
        VectorXd z_diff =z-z_pred;
        VectorXd new_x = K*(z_diff);
        x_.tail(12) += new_x.tail(12);
        VectorXd q_tmp = VecToQuaternionMath(new_x.head(3));
        x_.head(4) = MultiplyQuaternions(q_tmp,x_.head(4));
        
        P_ -= K*S*K.transpose();
        // double nis_lidar = (z_diff).transpose()*S.inverse()*(z_diff);
        // ofstream fout;
        // fout.open ("nis_stdAcc_" + std::to_string(std_a_) + "_stdYawDD_" + std::to_string(std_yawdd_) + ".csv",ios::app);
        // fout << nis_lidar << std::endl;
        // fout.close();
    }
    MatrixXd UKF::GenerateSigmaPoints()
    {
        VectorXd x_st = VectorXd(n_aug_);
        x_st.fill(0.0);
        x_st.head(n_x_) = x_;
    
        // dummy_var_size
        int n_dum = 6;//n_sig_-n_x_;
        MatrixXd Q_noise = MatrixXd(n_dum,n_dum);
        // for (int i=0;i<n_dum/2;i++) Q_noise(i,i) = std_w_meas_*std_w_meas_;
        // for (int i=n_dum/2;i<n_dum;i++) Q_noise(i,i) = std_a_meas_*std_a_meas_;

        // Note: P_aug is the covariance.
        // Covariance for orientation is given in terms of angles, so it has one less component.
        // Hence the size is (n_aug_-1,n_aug_-1)

        MatrixXd P_aug = P_;
        // P_aug.fill(0.0);
        // P_aug.topLeftCorner(n_x_-1, n_x_-1) = P_;
        // P_aug.bottomRightCorner(n_dum,n_dum) = Q_noise;

        //create sigma point matrix
        MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
    
        //calculate square root of P
        MatrixXd A = P_aug.llt().matrixL();

        //calculate sigma points ...
        //set sigma points as columns of matrix Xsig
        Xsig.col(0) = x_st;
    
        for (int i=0;i<A.cols();i++)
        {
            VectorXd x1(x_st.size());
            VectorXd x2(x_st.size());

            VectorXd del_s = sqrt(lambda_+n_aug_)*A.col(i);
            VectorXd del_q = VecToQuaternionMath(del_s.head(3));
            
            x1.head(4) = MultiplyQuaternions( del_q,x_st.head(4));
            x2.head(4) = MultiplyQuaternions(-del_q,x_st.head(4));

            x1.tail(n_aug_-4) = x_st.tail(n_aug_-4) + del_s.tail(n_aug_-3);
            x2.tail(n_aug_-4) = x_st.tail(n_aug_-4) - del_s.tail(n_aug_-3);

            Xsig.col(i+1) = x1;
            Xsig.col(i+1+n_aug_) = x2;
        }
        return Xsig;
    }
    VectorXd UKF::SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug)
    {
        MatrixXd Xsig_pred = MatrixXd(n_x_,n_sig_);

        double dt2 = delta_t*delta_t;
        
        VectorXd x_next = VectorXd(x_.size());
        x_next.fill(0.0);
        MatrixXd q_sig(4,n_sig_);
        for(int i=0; i < Xsig_aug.cols();i++)
        {
            VectorXd x_i = Xsig_aug.col(i);
            VectorXd quat = x_i.head(4);
            VectorXd    s = x_i.segment(4,6);
            VectorXd    v = x_i.segment(6,8);
            VectorXd    w = x_i.segment(8,10);
            VectorXd  acc = x_i.tail(3);

            VectorXd del_q = VecToQuaternionMath(w*delta_t);
            VectorXd new_q = MultiplyQuaternions(quat,del_q);
            q_sig.col(i) = new_q;
            s += v*delta_t + 0.5*acc*dt2;
            v += acc*delta_t;

            VectorXd x_new(n_x_);
            x_new << new_q,s,v,w,acc;
            Xsig_pred.col(i) = x_new;
            x_next += weights_(i)*Xsig_pred.col(i);
        }
        QuaternionAverage(q_sig,x_.head(4));
        x_next.head(4) = q_mean;
        //write result
        Xsig_pred_ = Xsig_pred;
        return x_next;
    }
    void UKF::QuaternionAverage(MatrixXd q_sig,VectorXd q_prev)
    {
        // hyper parameters for the algorithm
        double epsilon = 0.0001;
        int max_iter = 1000;

        VectorXd eps(4);
        eps << 0.0001,0.0001,0.0001,0.0001;

        MatrixXd qe(n_sig_, 4);
        VectorXd temp(4);
        temp.fill(0.0);
        int t = 0;
        while (t<max_iter)
        {
            VectorXd q_prev_inv = InverseQuaternion(q_prev);
            VectorXd err_mean(q_err.rows(),q_err.cols());
            err_mean.fill(0.0);
            for (int i=0;i<n_sig_;i++)
            {
                VectorXd qi = q_sig.col(i);
                qi.normalize();
                qe.col(i) = MultiplyQuaternions(qi,q_prev_inv);

                double qs = qe.col(i)(0);
                VectorXd qv = qe.col(i).segment(1,3);
                double qe_norm = qe.col(i).norm();
                
                if (qv.norm() < epsilon) q_err.col(i).fill(0.0);
                else
                {
                    if (qe_norm < epsilon) q_err.col(i).fill(0.0);
                    else
                    {
                        temp(0) = log(qe_norm);
                        qv.normalize();
                        temp.tail(3) = qv*acos(qs/qe_norm);
                        q_err.col(i) = 2*temp.tail(3);
                        double err_norm = q_err.col(i).norm();
                        double num = err_norm + M_PIl;
                        while (num >=  2*M_PIl) num -= 2*M_PIl;
                        while (num <= -2*M_PIl) num += 2*M_PIl;
                        q_err.col(i) = ((-M_PIl + num)/err_norm)*q_err.col(i);
                    }
                }
                err_mean += q_err.col(i)/double(n_sig_);
            }
            VectorXd temp2(4);
            temp2 << 0.0,err_mean/2.0;
            temp2 += eps;
            q_prev = MultiplyQuaternions(ExpQuaternion(temp2),q_prev);

            if (err_mean.norm() < epsilon) break;
        }
        q_mean = q_prev;
        return;
    }
    VectorXd UKF::VecToQuaternionMath(VectorXd angle)
    {
        assert(angle.size()==3);
        double len = angle.norm();
        angle.normalize();
        VectorXd q(4);
        q(0) = cos(len/2);
        q.tail(3) = sin(len/2)*angle;
        return q;
    }
    Quaterniond UKF::VecToQuaternion(VectorXd angle)
    {
        VectorXd a = VecToQuaternionMath(angle);
        Quaterniond q;
        q.w() = a(0);
        q.vec() = a.tail(3);
        return q;
    }
    VectorXd UKF::MultiplyQuaternions(VectorXd p, VectorXd q)
    {
        assert(p.size() == 4);
        assert(q.size() == 4);
        VectorXd res(p.size());
        res << p(0)*q(0) - p(1)*q(1) - p(2)*q(2) - p(3)*q(3), 
               p(0)*q(1) + p(1)*q(0) + p(2)*q(3) - p(3)*q(2),                          
               p(0)*q(2) + p(2)*q(0) + p(3)*q(1) - p(1)*q(3),
               p(0)*q(3) + p(3)*q(0) + p(1)*q(2) - p(2)*q(1);
        return res;
    }
    VectorXd UKF::InverseQuaternion(VectorXd q)
    {
        VectorXd q_inv(4);
        q_inv(0) = q(0)/ pow(q.norm(),2);
        q_inv(1) = -q(1)/ pow(q.norm(),2);
        q_inv(2) = -q(2)/ pow(q.norm(),2);
        q_inv(3) = -q(3)/ pow(q.norm(),2);
    
        return q_inv;
    }
    VectorXd UKF::ExpQuaternion(VectorXd q)
    {
        double qs = q(0);
        VectorXd qv(3);
        qv << q(1),q(2),q(3);
        double qv_norm = qv.norm();
        VectorXd exp_q(4);
        exp_q(0) = cos(qv_norm);
        qv.normalize();
        exp_q.segment(1,3) = sin(qv_norm)*qv;
        return exp(qs)*exp_q;
    }
}