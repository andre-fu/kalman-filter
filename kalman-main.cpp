#include <iostream>
#include "kalman.hpp"
#include "kalman.cpp"
#include <eigen3/Eigen/Dense>

using namespace std;

int main() {
    Eigen::Matrix<long double, 6, 6> P;
    Eigen::Matrix<long double, 6, 6> Q;
    Eigen::Matrix<long double, 4, 4> R;

    // THIS IS GIVEN ON YOUR DATA SHEET! 
    int ra = pow(10, 2); //noise in acccel
    int rp = pow(100, 2); //noise in position
    long double dt = 0.1;
    R << rp, 0, 0, 0,
         0, rp, 0, 0,
         0,  0, ra, 0,
         0,  0,  0, ra;
    
    // play with this noise covar matrix
    P << 100, 0, 0,   0,   0,   0, 
         0, 100, 0,   0,   0,   0, 
         0,   0, 10,   0,    0,    0, 
         0,   0,   0,  10,  0,  0, 
         0,   0,   0,    0, 1,  0, 
         0,   0,   0,    0,   0,  1;

    long double sa = 0.001; //variance in process noise from accel
    

    Eigen::Matrix<long double, 6, 1> G;
    G << 0.5*pow(dt, 2), 0.5*pow(dt, 2), dt, dt, 1, 1;
    G = G.transpose();
    Q = G*G.transpose()*pow(sa, 2);
 
    // cout << "G: \n" << GG << endl;

    // cout << "R: \n" << R << endl;
    // cout << "P: \n" << P << endl;
    // cout << "Q: \n" << Q << endl;
    // cout << typeid(P).name() << endl;
    // cout << typeid(Q).name() << endl;
    // cout << typeid(R).name() << endl;
    KalmanFilter kf(R, Q, P);


}