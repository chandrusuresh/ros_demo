#ifndef SENSOR_NOISE_H
#define SENSOR_NOISE_H

#include <random>
#include <chrono>

using namespace std;

namespace ros_demo
{
    struct NoiseParameters
    {
        public:
        double bias_correlation_time = 1000;
        double white_noise_density = 1;
        double random_walk_density = 1;
        double turn_on_bias_density = 1;
    };
    class NoiseModel
    {
        public:
        NoiseModel(){init();}
        NoiseModel(double tau,double std_wn, double std_rw,double std_turn_on_bias)
        {
            init();
            params.bias_correlation_time = tau;
            params.white_noise_density = std_wn;
            params.random_walk_density = std_rw;
            params.turn_on_bias_density = std_turn_on_bias;
            turn_on_bias = params.turn_on_bias_density* standard_normal_distribution_(random_generator_);
        }
        NoiseModel(NoiseParameters p)
        {
            init();
            params=p;
            turn_on_bias = params.turn_on_bias_density* standard_normal_distribution_(random_generator_);
        }
        ~NoiseModel(){}
        double sample(double dt)
        {
            assert(dt > 0);
            double noise = 0.0;

            double std_wn_d = params.white_noise_density/sqrt(dt);

            double tau = params.bias_correlation_time;
            double var_rw_d = pow(params.random_walk_density,2)*tau/2*(1-exp(-2*dt/tau));
            double phi_d = exp(-1.0/tau*dt);
            bias = phi_d*bias + sqrt(var_rw_d)* standard_normal_distribution_(random_generator_);

            noise = bias + turn_on_bias + std_wn_d* standard_normal_distribution_(random_generator_);
            return noise;
        }
        private:
        NoiseParameters params;
        double bias = 0.0;
        default_random_engine random_generator_;
        normal_distribution<double> standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
        double turn_on_bias;
        void init()
        {
            unsigned seed = chrono::system_clock::now().time_since_epoch().count();
            random_generator_.seed(seed);
        }        
    };
}

#endif // SENSOR_NOISE_H