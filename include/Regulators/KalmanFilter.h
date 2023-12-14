#include <Eigen3/Eigen/Dense>

namespace kalman
{

    class KalmanFilter
    {
        public:

            KalmanFilter(){};

            KalmanFilter(Eigen::Matrix<double, 4, 4> A_in, Eigen::Matrix<double, 4, 4> B_in, Eigen::Matrix<double, 2, 4> C_in,
                         Eigen::Matrix<double, 4, 4> Q_in, Eigen::Matrix<double, 2, 2> R_in, Eigen::Matrix<double, 2, 1> w_in,
                         Eigen::Matrix<double, 4, 4> P0_in, Eigen::Matrix<double, 4, 1> x0_in);

            KalmanFilter(const KalmanFilter& filter);

            // function used for calculating prediction estimate
            Eigen::Matrix<double, 4, 1> predictEstimate(Eigen::Matrix<double, 4, 1> extInput);

            // function used for updating estimate
            void updateEstimate(Eigen::Matrix<double, 2, 1> measurement);

            // function to reset kalman matrices in case of turning off automatic landing 
            void resetKalman();

            // setting A matrix with current time difference
            void update_AQmatrix(double delta_t);


        private:

            Eigen::Matrix<double, 4, 4> m_A; 
            Eigen::Matrix<double, 4, 4> m_B;
            Eigen::Matrix<double, 2, 4> m_C;
            Eigen::Matrix<double, 4, 4> m_Q;
            Eigen::Matrix<double, 2, 2> m_R;
            Eigen::Matrix<double, 4, 4> m_P0;

            // initial state
            Eigen::Matrix<double, 4, 1> m_x0;

            // process noice variances
            Eigen::Matrix<double, 2, 1> m_w;

            // posteriori estimate
            Eigen::Matrix<double, 4, 1> m_estimateAposteriori;

            // priori estimate
            Eigen::Matrix<double, 4, 1> m_estimateApriori;

            // posteriori estimation error covariance matrix
            Eigen::Matrix<double, 4, 4> m_covarianceAposteriori;

            // priori estimation error covariance matrix
            Eigen::Matrix<double, 4, 4> m_covarianceApriori;

            // gain matrix K
            Eigen::Matrix<double, 4, 2> m_kalmanGain;

            // prediction error
            Eigen::Matrix<double, 2, 1> m_error;

            // m - input dimension, n- state dimension, r-output dimension
            unsigned int m, n, r; 
    };


} // namespace kalman