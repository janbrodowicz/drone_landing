#include <cmath>
#include "Regulators/KalmanFilter.h"


using namespace kalman;


template<int State, int Input, int Measure>
KalmanFilter<State, Input, Measure>::KalmanFilter(Eigen::Matrix<double, State, State> A_in, Eigen::Matrix<double, State, Input> B_in, Eigen::Matrix<double, Measure, State> C_in,
                                                  Eigen::Matrix<double, State, 1> G_in, Eigen::Matrix<double, State, State> Q_in, Eigen::Matrix<double, Measure, Measure> R_in,
                                                  Eigen::Matrix<double, 2, 1> w_in, Eigen::Matrix<double, State, State> P0_in, Eigen::Matrix<double, State, 1> x0_in)
{
    // initialize matrices
    m_A = A_in; m_B = B_in; m_C = C_in; m_G = G_in; m_Q = Q_in;
    m_R = R_in; m_P0 = P0_in; m_w = w_in; m_x0 = x0_in;

    // extract the appropriate dimensions
    n = m_A.rows();   m = m_B.cols(); r = m_C.rows();

    // initial guess
    m_estimateAposteriori = m_x0;

    // initial estimate covariance
    m_covarianceAposteriori = m_P0;

    // set sizes of the rest of matrices and set with zeros
    // m_estimateApriori.resize(n, 1);
    m_estimateApriori.setZero();

    // m_covarianceApriori.resize(n, n);
    m_covarianceApriori.setZero();

    // m_kalmanGain.resize(n, r);
    m_kalmanGain.setZero();

    // m_error.resize(r, 1);
    m_error.setZero();
}

template<int State, int Input, int Measure>
KalmanFilter<State, Input, Measure>::KalmanFilter(const KalmanFilter<State, Input, Measure>& filter)
{
    // initialize matrices
    m_A = filter.m_A; m_B = filter.m_B; m_C = filter.m_C; m_G = filter.m_G; m_Q = filter.m_Q;
    m_R = filter.m_R; m_P0 = filter.m_P0; m_w = filter.m_w; m_x0 = filter.m_x0;

    // extract the appropriate dimensions
    n = m_A.rows();   m = m_B.cols(); r = m_C.rows();

    // initial guess
    m_estimateAposteriori = m_x0;

    // initial estimate covariance
    m_covarianceAposteriori = m_P0;

    // set sizes of the rest of matrices and set with zeros
    // m_estimateApriori.resize(n, 1);
    m_estimateApriori.setZero();

    // m_covarianceApriori.resize(n, n);
    m_covarianceApriori.setZero();

    // m_kalmanGain.resize(n, r);
    m_kalmanGain.setZero();

    // m_error.resize(r, 1);
    m_error.setZero();
}

template<int State, int Input, int Measure>
Eigen::Matrix<double, State, 1> KalmanFilter<State, Input, Measure>::predictEstimate(Eigen::Matrix<double, Input, 1> extInput)
{
    // calculate priori estimate
    m_estimateApriori = m_A * m_estimateAposteriori + m_B * extInput + m_G;

    // calculate priori covariance
    m_covarianceApriori = m_A * m_covarianceAposteriori * (m_A.transpose()) + m_Q;

    return m_estimateApriori;
}

template<int State, int Input, int Measure>
void KalmanFilter<State, Input, Measure>::updateEstimate(Eigen::Matrix<double, Measure, 1> measurement)
{
    // additional matrix for kalman gain calculation
    Eigen::MatrixXd Sk;
    Sk.resize(r, r);
    Sk = (m_C * m_covarianceApriori * m_C.transpose()) + m_R;
    Sk.inverse();

    // kalman gain
    m_kalmanGain = m_covarianceApriori * (m_C.transpose()) * Sk;

    // error
    m_error = measurement - (m_C * m_estimateApriori);

    // calculate posteriori estimate
    m_estimateAposteriori = m_estimateApriori + (m_kalmanGain * m_error);

    // additional matrices for a posteriori covariance calculation
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(n, n); 
    Eigen::MatrixXd Iminus;
    Iminus.resize(n, n);
    Iminus = I - (m_kalmanGain * m_C);

    // calculate posteriori covariance
    m_covarianceAposteriori = Iminus * m_covarianceApriori;
}

template<int State, int Input, int Measure>
void KalmanFilter<State, Input, Measure>::resetKalman()
{
    // reset posteriori estimate matrix with initial guess
    m_estimateAposteriori = m_x0;

    // reset posteriori matrix with initial covariance
    m_covarianceAposteriori = m_P0;

    // reseting matrices
    m_estimateApriori.setZero();

    m_covarianceApriori.setZero();

    m_kalmanGain.setZero();

    m_error.setZero();
}

template<int State, int Input, int Measure>
void KalmanFilter<State, Input, Measure>::updateInitial(Eigen::Matrix<double, State, 1> x0_in)
{
    m_estimateAposteriori = x0_in;
}

// template<int State, int Input, int Measure>
// void KalmanFilter<State, Input, Measure>::update_AQmatrix(double delta_t)
// {
//     m_A(0, 2) = delta_t;
//     m_A(0, 4) = std::pow(delta_t, 2) / 2;
//     m_A(1, 3) = delta_t;
//     m_A(0, 5) = std::pow(delta_t, 2) / 2;
//     m_A(2, 4) = delta_t;
//     m_A(3, 5) = delta_t;

//     m_Q(0, 0) = (std::pow(delta_t, 6) / 9) * m_w(0);
//     m_Q(0, 2) = (std::pow(delta_t, 5) / 6) * m_w(0);
//     m_Q(0, 4) = (std::pow(delta_t, 4) / 3) * m_w(0);

//     m_Q(1, 1) = (std::pow(delta_t, 6) / 9) * m_w(1);
//     m_Q(1, 3) = (std::pow(delta_t, 5) / 6) * m_w(1);
//     m_Q(1, 5) = (std::pow(delta_t, 4) / 3) * m_w(1);

//     m_Q(2, 0) = (std::pow(delta_t, 5) / 6) * m_w(0);
//     m_Q(2, 2) = (std::pow(delta_t, 4) / 4) * m_w(0);
//     m_Q(2, 4) = (std::pow(delta_t, 3) / 2) * m_w(0);

//     m_Q(3, 1) = (std::pow(delta_t, 5) / 6) * m_w(1);
//     m_Q(3, 3) = (std::pow(delta_t, 4) / 4) * m_w(1);
//     m_Q(3, 5) = (std::pow(delta_t, 3) / 2) * m_w(1);

//     m_Q(4, 0) = (std::pow(delta_t, 4) / 3) * m_w(0);
//     m_Q(4, 2) = (std::pow(delta_t, 3) / 2) * m_w(0);
//     m_Q(4, 4) = std::pow(delta_t, 2) * m_w(0);

//     m_Q(5, 1) = (std::pow(delta_t, 4) / 3) * m_w(1);
//     m_Q(5, 3) = (std::pow(delta_t, 3) / 2) * m_w(1);
//     m_Q(5, 5) = std::pow(delta_t, 2) * m_w(1);

//     m_B(0, 0) = (std::pow(delta_t, 3) / 3);
//     m_B(1, 1) = (std::pow(delta_t, 3) / 3);
//     m_B(2, 2) = (std::pow(delta_t, 2) / 2);
//     m_B(3, 3) = (std::pow(delta_t, 2) / 2);
//     m_B(4, 4) = delta_t;
//     m_B(5, 5) = delta_t;
// }

template<int State, int Input, int Measure>
void KalmanFilter<State, Input, Measure>::update_AQmatrix(double delta_t)
{
    m_A(0, 2) = delta_t;
    m_A(1, 3) = delta_t;

    m_Q(0, 0) = (std::pow(delta_t, 4) / 4) * m_w(0);
    m_Q(0, 2) = (std::pow(delta_t, 3) / 2) * m_w(0);

    m_Q(1, 1) = (std::pow(delta_t, 4) / 4) * m_w(1);
    m_Q(1, 3) = (std::pow(delta_t, 3) / 2) * m_w(1);

    m_Q(2, 0) = (std::pow(delta_t, 3) / 2) * m_w(0);
    m_Q(2, 2) = (std::pow(delta_t, 2)) * m_w(0);

    m_Q(3, 1) = (std::pow(delta_t, 3) / 2) * m_w(1);
    m_Q(3, 3) = (std::pow(delta_t, 2)) * m_w(1);

    m_B(0, 0) = (std::pow(delta_t, 2) / 2);
    m_B(1, 1) = (std::pow(delta_t, 2) / 2);
    m_B(2, 2) = delta_t;
    m_B(3, 3) = delta_t;
}

// template<int State, int Input, int Measure>
// void KalmanFilter<State, Input, Measure>::update_AQmatrix(double delta_t)
// {
//     m_Q(0, 0) = std::pow(delta_t, 2) * m_w(0);
//     m_Q(1, 1) = std::pow(delta_t, 2) * m_w(1);

//     m_B(0, 0) = delta_t;
//     m_B(1, 1) = delta_t;
// }

template class KalmanFilter<2, 2, 2>;
template class KalmanFilter<4, 4, 4>;
template class KalmanFilter<4, 4, 2>;
template class KalmanFilter<6, 6, 6>;