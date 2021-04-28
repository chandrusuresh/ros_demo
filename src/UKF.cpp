#include "UKF.h"
using namespace std;
using namespace Eigen;

namespace ros_demo
{
    UKF::UKF()
    {
        // initial state vector
        x_ = VectorXd(5);

        // initial covariance matrix
        P_ = MatrixXd(5, 5);

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

        // Dimension of state vector: s,v,q
        n_x_ = 10;

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

        Xsig_pred_ = MatrixXd(n_x_,n_sig_);
    }
    UKF::~UKF(){}
    void UKF::Prediction(double delta_t)
    {
        double dt2 = delta_t*delta_t;

        //predict sigma points
        MatrixXd Xsig_aug = GenerateSigmaPoints();

        //write predicted sigma points into right column
        VectorXd x_next = SigmaPointPrediction(delta_t,Xsig_aug);

        MatrixXd P_next = MatrixXd(n_x_,n_x_);
        P_next.fill(0.0);
        for (int i=0; i<n_sig_;i++)
        {
            VectorXd del_x = Xsig_pred_.col(i)-x_next;
            while (del_x(3)> M_PI) del_x(3)-=2.*M_PI;
            while (del_x(3)<-M_PI) del_x(3)+=2.*M_PI;
    //        while (del_x(4)> M_PI) del_x(4)-=2.*M_PI;
    //        while (del_x(4)<-M_PI) del_x(4)+=2.*M_PI;
            P_next += weights_(i)*del_x*del_x.transpose();
        }
        x_ = x_next;
        P_ = P_next;
    }
    MatrixXd UKF::GenerateSigmaPoints()
    {
        VectorXd x_st = VectorXd(n_aug_);
        x_st.fill(0.0);
        x_st.head(n_x_) = x_;
    
        // dummy_var_size
        int n_dum = n_sig_-n_x_;
        MatrixXd Q_noise = MatrixXd(n_dum,n_dum);
        for (int i=0;i<n_dum/2;i++) Q_noise(i,i) = std_w_meas_*std_w_meas_;
        for (int i=n_dum/2;i<n_dum;i++) Q_noise(i,i) = std_a_meas_*std_a_meas_;

        // Note: P_aug is the covariance.
        // Covariance for orientation is given in terms of angles, so it has one less component.
        // Hence the size is (n_aug_-1,n_aug_-1)

        MatrixXd P_aug = MatrixXd(n_aug_-1,n_aug_-1);
        P_aug.fill(0.0);
        P_aug.topLeftCorner(n_x_-1, n_x_-1) = P_;
        P_aug.bottomRightCorner(n_dum,n_dum) = Q_noise;

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
            VectorXd del_q = AngleToQuaternionMath(del_s.head(3));
            
            x1.head(4) = MultiplyQuaternions(x_st.head(4), del_q);
            x2.head(4) = MultiplyQuaternions(x_st.head(4),-del_q);

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
        
        for(int i=0; i < Xsig_aug.cols();i++)
        {
            VectorXd x_i = Xsig_aug.col(i);
            VectorXd quat = x_i.head(4);
            VectorXd    s = x_i.segment(4,6);
            VectorXd    v = x_i.segment(6,8);
            VectorXd    w = x_i.segment(8,10);
            VectorXd  acc = x_i.tail(3);

            VectorXd del_q = AngleToQuaternion(w*delta_t);
            VectorXd

            VectorXd x_state = (Xsig_aug.col(i)).head(5);//.block(0,i,5,0);
            VectorXd noise = (Xsig_aug.col(i)).tail(2);//.block(4,i,2,0);

            VectorXd vec_1 = VectorXd(x_state.size());
            VectorXd vec_2 = VectorXd(x_state.size());
            
            vec_2(0) = noise(0)*cos(x_state(3))*dt2/2;
            vec_2(1) = noise(0)*sin(x_state(3))*dt2/2;
            vec_2(2) = noise(0)*delta_t;
            vec_2(3) = noise(1)*dt2/2;
            vec_2(4) = noise(1)*delta_t;
            
            if (fabs(x_state(4)) < 1E-6)
            {
                vec_1(0) = x_state(2)*cos(x_state(3))*delta_t;
                vec_1(1) = x_state(2)*sin(x_state(3))*delta_t;
            }
            else
            {
                double phi_k1 = x_state(3) + x_state(4)*delta_t;
                vec_1(0) = x_state(2)/x_state(4)*( sin(phi_k1) - sin(x_state(3)));
                vec_1(1) = x_state(2)/x_state(4)*(-cos(phi_k1) + cos(x_state(3)));
            }
            vec_1(2) = 0;
            vec_1(3) = x_state(4)*delta_t;
            vec_1(4) = 0;
            VectorXd x1 = x_state + vec_1 + vec_2;
    //        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
    //        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
    //        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
    //        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
            Xsig_pred.col(i) = x1;
            x_next += weights_(i)*Xsig_pred.col(i);
            
        }
    //    while (x_next(3)> M_PI) x_next(3)-=2.*M_PI;
    //    while (x_next(3)<-M_PI) x_next(3)+=2.*M_PI;
    //    while (x_next(4)> M_PI) x_next(4)-=2.*M_PI;
    //    while (x_next(4)<-M_PI) x_next(4)+=2.*M_PI;

        //write result
        Xsig_pred_ = Xsig_pred;
        return x_next;
    }
    VectorXd UKF::AngleToQuaternionMath(VectorXd angle)
    {
        assert(angle.size()==3);
        double len = angle.norm();
        angle.normalize();
        VectorXd q(4);
        q(0) = cos(len/2);
        q.tail(3) = sin(len/2)*angle;
        return q;
    }
    Quaterniond UKF::AngleToQuaternion(VectorXd angle)
    {
        VectorXd a = AngleToQuaternionMath(angle);
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
}