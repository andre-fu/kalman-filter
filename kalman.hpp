#include <eigen3/Eigen/Dense>

// X = ( x y x' y' x'' y'')^T

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
        Eigen::MatrixXf F; //aliased to A in other texts 
        // Eigen::MatrixXd & B; // assume no control
        Eigen::MatrixXf P;
        Eigen::MatrixXf Q;

        // Eigen::VectorXd & u; // assume no control input 

        Eigen::VectorXf X_new;
        Eigen::VectorXf X_prev;


        //update
        Eigen::MatrixXf H;
        Eigen::MatrixXf K;
        Eigen::MatrixXf R; 
        Eigen::MatrixXf z;

        
        long t = 0;     // seconds
        long dt  = 0.1; // seconds
        Eigen::MatrixXd I; //Id 


    public:
        KalmanFilter(
            const Eigen::MatrixXd & R,
            const Eigen::MatrixXd & Q,
            const Eigen::MatrixXd & P
        );
        ~KalmanFilter();

        // intial t0 & intial state matrix
        void init(long t0, long dt, const Eigen::MatrixXd & X0);
        void predict();
        void update(const Eigen::VectorXd & z);  
        Eigen::VectorXd state();




};