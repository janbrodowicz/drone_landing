#include <Eigen3/Eigen/Dense>

namespace kalman
{

    class KalmanFilter
    {
        public:

            KalmanFilter(){};

            KalmanFilter(Eigen::MatrixXd A_in, Eigen::MatrixXd B_in, Eigen::MatrixXd C_in,
                         Eigen::MatrixXd Q_in, Eigen::MatrixXd R_in, Eigen::MatrixXd P0_in,
                         Eigen::MatrixXd x0_in);

            KalmanFilter(const KalmanFilter& filter);

            // function used for calculating prediction estimate
            void predictEstimate(Eigen::MatrixXd extInput, Eigen::MatrixXd& estimateApriori, Eigen::MatrixXd& covarianceApriori);

            // function used for updating estimate
            void updateEstimate(Eigen::MatrixXd measurement, Eigen::MatrixXd& estimateAposteriori, Eigen::MatrixXd& covarianceAposteriori);

            // function to reset kalman matrices in case of turning off automatic landing 
            void resetKalman();

            // setting A matrix with current time difference
            void update_Amatrix(double delta_t);

        private:

            Eigen::MatrixXd A, B, C, Q, R, P0;

            // initial state
            Eigen::MatrixXd x0;

            // posteriori estimate
            Eigen::MatrixXd estimateAposteriori;

            // priori estimate
            Eigen::MatrixXd estimateApriori;

            // posteriori estimation error covariance matrix
            Eigen::MatrixXd covarianceAposteriori;

            // priori estimation error covariance matrix
            Eigen::MatrixXd covarianceApriori;

            // gain matrix K
            Eigen::MatrixXd kalmanGain;

            // prediction error
            Eigen::MatrixXd error;

            // m - input dimension, n- state dimension, r-output dimension
            unsigned int m, n, r; 
    };


} // namespace kalman