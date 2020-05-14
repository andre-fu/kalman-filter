#include <eigen3/Eigen/Dense>
#pragma once
// X = ( x y x' y')^T

// PREDICT:
//     X_k+1 = F*Xk + B*u
//     P_k+1 = F*P_k*F^T + Q

// UPDATE:
//     K_k = P_k*H^T (H*P_k *H^T + R )^ -1
//     X_k = X_k + K_k(z_k - H*X_k)
//     P_k = (Id - K_k *H) * P_k



class KalmanFilter {
    private:
        //predict
        Eigen::Matrix<long double, 4, 4> F; //aliased to A in other texts 
        // Eigen::MatrixXd & B; // assume no control
        Eigen::Matrix<long double, 4, 4> P;
        Eigen::Matrix<long double, 4, 4> Q;

        // Eigen::VectorXd & u; // assume no control input 

        Eigen::Matrix<long double, 4, 1> X_new;
        Eigen::Matrix<long double, 4, 1> X_prev;


        //update
        Eigen::Matrix<long double, 2, 4> H;
        Eigen::Matrix<long double, 4, 2> K;
        Eigen::Matrix<long double, 2, 2> R; 
        Eigen::Matrix<long double, 2, 1> z;

        
        long double t;     // seconds
        long double dt; // seconds
        Eigen::Matrix<long double, 4, 4> I; //Id 


    public:
        KalmanFilter(
            // const Eigen::Matrix<long double, 6, 6> R,
            // const Eigen::Matrix<long double, 6, 6> Q,
            // const Eigen::Matrix<long double, 6, 6> P
            const Eigen::Matrix<long double, 2, 2> R,
            const Eigen::Matrix<long double, 4, 4> Q,
            const Eigen::Matrix<long double, 4, 4> P
        );
        
        ~KalmanFilter();

        // intial t0 & intial state matrix
        void init(long double t0, long double dt, const Eigen::Matrix<long double, 4, 1> X0);
        void predict();
        void update(const Eigen::Matrix<long double, 2, 1> z);  
        Eigen::Matrix<long double, 4, 1> state();




};

