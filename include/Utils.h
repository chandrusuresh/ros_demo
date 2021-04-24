#ifndef UTILS_H
#define UTILS_H

#include <random>
#include <Eigen/Dense>
#include "eigenmvn.h"

using namespace Eigen;

namespace ros_demo
{
    class Gaussian
    {
        public:
        Gaussian(int n=1)
        {
            static_assert(n>0);
            size = n;
            mu  = new VectorXd(n);
            cov = new MatrixXd(n,n);
        }
        ~Gaussian(){}
        void setMean(VectorXd x)
        {
            static_assert(size == x.size());
            mu = x;
        }
        void setCovariance(MatrixXd P)
        {
            covar_check = false;
            static_assert(size == P.rows());
            static_assert(size == P.cols());
            cov = P;
            static_assert(checkCovariance());
            covar_check = true;
        }
        void setSampler()
        {
            static_assert(n == x.size());
            Sampler = new EigenMultivariateNormal(mu,cov);
        }
        MatrixXd sample(int num)
        {
            static_assert(size > 0);
            static_assert(size == mu.size());
            if (!covar_check) static_assert(checkCovariance());
            return Sampler.samples(num);
        }
        private:
        bool covar_check = false;
        int size;
        VectorXd mu;
        MatrixXd cov;
        EigenMultivariateNormal<double> Sampler;
        void checkCovariance()
        {
            for (int i=0;i<size;i++)
            {
                if (cov[i,i] == 0) return false;
                for (int j=i+1;j<size;j++)
                {
                    if (cov[i,j] != cov[j,i]) return false;
                }
            }
            return true;
        }
    }
};
#endif // UTILS_H