#include <iostream>
#include <vector>
#include "kalman.hpp"
#include "kalman.cpp"
#include <eigen3/Eigen/Dense>

using namespace std;

int main() {
    Eigen::Matrix<long double, 4, 4> P;
    Eigen::Matrix<long double, 4, 4> Q;
    Eigen::Matrix<long double, 2, 2> R;

    // THIS IS GIVEN ON YOUR DATA SHEET! 
    int ra = pow(10, 2); //noise in x, y (its the same)
    long double t = 0;
    long double dt = 0.75;
    R << ra, 0,
         0, ra;
    
    // play with this noise covar matrix
    P << 1000, 0, 0,     0,  
         0, 1000, 0,     0,  
         0,   0, 1000,   0,  
         0,   0,   0,  1000;

    long double sa = 8.8; //variance in process noise from accel
    

    Eigen::Matrix<long double, 4, 1> G;
    G << 0.5*pow(dt, 2), 0.5*pow(dt, 2), dt, dt;
    G = G.transpose();
    Q = G*G.transpose()*pow(sa, 2);
 
    // cout << "G: \n" << GG << endl;

    // cout << "R: \n" << R << endl;
    // cout << "P: \n" << P << endl;
    // cout << "Q: \n" << Q << endl;
    // cout << typeid(P).name() << endl;
    // cout << typeid(Q).name() << endl;
    // cout << typeid(R).name() << endl;
    Eigen::Matrix<long double, 2, 1> z_measure;
    vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
    };
    KalmanFilter kf(R, Q, P);
    Eigen::Matrix<long double, 4, 1> X0;
    X0.setZero();

    kf.init(t, dt, X0);
    bool stop = false;
    while(stop == false){
        t += dt;
        int i = 0;
        kf.predict();
        z_measure << measurements[i], measurements[i+1];
        kf.update(z_measure);
        cout << "at t: " << t << " State, X: \n" << kf.state().transpose() << endl;
        i++;
        if (i == measurements.size() -1){
            stop = true;
        }
    }
}