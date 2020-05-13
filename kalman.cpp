#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kalman.hpp"
#include <math.h>  


KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & P
): R(R), Q(Q), P(P) {
    F(6,6); P(6,6); Q(6,6); H(6,6); R(4,4); z(4,1); 
    I = Eigen::MatrixXd::Identity(6, 6);
    
    X_new(6); X_prev(6);
    // F << 1, 0, dt, 0, pow(0.5*dt, 2), 0,
    //      0, 1, 0, dt, 0,              pow(0.5*dt, 2),
    //      0, 0, 1, 0,  dt,             0,
    //      0, 0, 0, 1,  0,              dt,
    //      0, 0, 0, 0,  1,              0, 
    //      0, 0, 0, 0,  0,                1;


    // H << 1, 0, 0, 0, 0, 0,
    //      0, 1, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0, 
    //      0, 0, 0, 0, 1, 0,
    //      0, 0, 0, 0, 0, 1;

    X_prev.setZero();
    X_new.setZero();

}

void KalmanFilter::init(long t0, long dt, const Eigen::MatrixXd & X0) {
    X_prev = X0;
    t = t0;
    dt = dt;
}

void KalmanFilter::predict() {
    X_new = F*X_prev;
    P = F*P*F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd & z) {
    K = P*H.transpose() * (H*P*H.transpose() + R).inverse();
    X_new = X_new + K*(z - H*X_new);
    P = (I - K*H)*P;
    X_prev = X_new;

    t += dt;
}
Eigen::VectorXd KalmanFilter::state() {
    return X_new;
}

int main() {
}