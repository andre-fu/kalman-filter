#include <iostream>
//#include <eigen3/Eigen/Dense>
#include "kalman.hpp"
#include <math.h>  
using namespace std;


KalmanFilter::KalmanFilter(
    const Eigen::Matrix<long double, 4, 4> R,
    const Eigen::Matrix<long double, 6, 6> Q,
    const Eigen::Matrix<long double, 6, 6> P
): R(R), Q(Q), P(P) {
    cout << "from class init" << endl;
    // //F(6,6); P(6,6); Q(6,6); H(6,6); R(4,4); z(4,1); 
    I = Eigen::MatrixXf::Identity(6, 6);
    cout << "I: \n" << I << endl;
    // cout << "R: \n" << R << endl;
    // cout << "P: \n" << P << endl;
    // cout << "Q: \n" << Q << endl;
    // X_new(6); X_prev(6);
    // // F << 1, 0, dt, 0, 0.5*pow(dt, 2), 0,
    // //      0, 1, 0, dt, 0,              0.5*pow(dt, 2),
    // //      0, 0, 1, 0,  dt,             0,
    // //      0, 0, 0, 1,  0,              dt,
    // //      0, 0, 0, 0,  1,              0, 
    // //      0, 0, 0, 0,  0,                1;


    // // H << 1, 0, 0, 0, 0, 0,
    // //      0, 1, 0, 0, 0, 0,
    // //      0, 0, 0, 0, 0, 0, 
    // //      0, 0, 0, 0, 1, 0,
    // //      0, 0, 0, 0, 0, 1;

    // X_prev.setZero();
    // X_new.setZero();

    // t = 0;
    // dt = 0.1;
}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::init(long t0, long dt, const Eigen::VectorXf X0) {
    X_prev = X0;
    t = t0;
    dt = dt;
}

// void KalmanFilter::predict() {
//     X_new = F*X_prev;
//     P = F*P*F.transpose() + Q;
// }

// void KalmanFilter::update(const Eigen::VectorXf z) {
//     K = P*H.transpose() * (H*P*H.transpose() + R).inverse();
//     X_new = X_new + K*(z - H*X_new);
//     P = (I - K*H)*P;
//     X_prev = X_new;

//     t += dt;
// }
// Eigen::VectorXf KalmanFilter::state() {
//     return X_new;
// }

// int main () {}