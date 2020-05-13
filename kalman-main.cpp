#include <iostream>
#include "kalman.hpp"
#include <eigen3/Eigen/Dense>

using namespace std;

int main() {
    Eigen::Matrix<long double, 6, 6> P;
    Eigen::Matrix<long double, 6,6> Q;
    Eigen::MatrixXf R(4,4);
    int ra = pow(10, 2); //noise in acccel
    int rp = pow(100, 2); //noise in position

    long double dt = 0.1;
    // THIS IS GIVEN ON YOUR DATA SHEET! 
    R << rp, 0, 0, 0,
         0, rp, 0, 0,
         0,  0, ra, 0,
         0,  0,  0, ra;
    
    // play with this
    P << 100, 0, 0,   0,   0,   0, 
         0, 100, 0,   0,   0,   0, 
         0,   0, 10,   0,    0,    0, 
         0,   0,   0,  10,  0,  0, 
         0,   0,   0,    0, 1,  0, 
         0,   0,   0,    0,   0,  1;

    long double sa = 0.001; //variance in process noise from accel
    

    Eigen::Matrix<long double, 6, 1> G;
    G << pow(0.5*dt, 2), pow(0.5*dt, 2), dt, dt, 1, 1;
    G = G.transpose();
    Q = G*G.transpose()*pow(sa, 2);
 
    // cout << "G: \n" << GG << endl;

    cout << "R: \n" << R << endl;
    cout << "P: \n" << P << endl;
    cout << "Q: \n" << Q << endl;

    //KalmanFilter kf(R, Q, P);

}