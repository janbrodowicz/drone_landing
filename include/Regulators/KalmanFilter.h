#include <Eigen3/Eigen/Dense>

namespace kalman
{

    template<int State, int Input, int Measure>
    class KalmanFilter
    {
        public:

            KalmanFilter(){};

            KalmanFilter(Eigen::Matrix<double, State, State> A_in, Eigen::Matrix<double, State, Input> B_in, Eigen::Matrix<double, Measure, State> C_in,
                         Eigen::Matrix<double, State, 1> G_in, Eigen::Matrix<double, State, State> Q_in, Eigen::Matrix<double, Measure, Measure> R_in, 
                         Eigen::Matrix<double, 2, 1> w_in, Eigen::Matrix<double, State, State> P0_in, Eigen::Matrix<double, State, 1> x0_in);

            KalmanFilter(const KalmanFilter<State, Input, Measure>& filter);

            // function used for calculating prediction estimate
            Eigen::Matrix<double, State, 1> predictEstimate(Eigen::Matrix<double, Input, 1> extInput);

            // function used for updating estimate
            void updateEstimate(Eigen::Matrix<double, Measure, 1> measurement);

            // function to reset kalman matrices in case of turning off automatic landing 
            void resetKalman();

            // setting A matrix with current time difference
            void update_AQmatrix(double delta_t);


        private:

            Eigen::Matrix<double, State, State> m_A; 
            Eigen::Matrix<double, State, Input> m_B;
            Eigen::Matrix<double, Measure, State> m_C;
            Eigen::Matrix<double, State, 1> m_G;
            Eigen::Matrix<double, State, State> m_Q;
            Eigen::Matrix<double, Measure, Measure> m_R;
            Eigen::Matrix<double, State, State> m_P0;

            // initial state
            Eigen::Matrix<double, State, 1> m_x0;

            // process noice variances
            Eigen::Matrix<double, 2, 1> m_w;

            // posteriori estimate
            Eigen::Matrix<double, State, 1> m_estimateAposteriori;

            // priori estimate
            Eigen::Matrix<double, State, 1> m_estimateApriori;

            // posteriori estimation error covariance matrix
            Eigen::Matrix<double, State, State> m_covarianceAposteriori;

            // priori estimation error covariance matrix
            Eigen::Matrix<double, State, State> m_covarianceApriori;

            // gain matrix K
            Eigen::Matrix<double, State, Measure> m_kalmanGain;

            // prediction error
            Eigen::Matrix<double, Measure, 1> m_error;

            // m - input dimension, n- state dimension, r-output dimension
            unsigned int m, n, r; 
    };


} // namespace kalman