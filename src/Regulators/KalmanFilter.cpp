#include <cmath>
#include "Regulators/KalmanFilter.h"


using namespace kalman;


KalmanFilter::KalmanFilter(Eigen::Matrix<double, 4, 4> A_in, Eigen::Matrix<double, 4, 4> B_in, Eigen::Matrix<double, 2, 4> C_in,
                           Eigen::Matrix<double, 4, 4> Q_in, Eigen::Matrix<double, 2, 2> R_in, Eigen::Matrix<double, 2, 1> w_in,
                           Eigen::Matrix<double, 4, 4> P0_in, Eigen::Matrix<double, 4, 1> x0_in)
{
    // initialize matrices
    m_A = A_in; m_B = B_in; m_C = C_in; m_Q = Q_in;
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

KalmanFilter::KalmanFilter(const KalmanFilter& filter)
{
    // initialize matrices
    m_A = filter.m_A; m_B = filter.m_B; m_C = filter.m_C; m_Q = filter.m_Q;
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

Eigen::Matrix<double, 4, 1> KalmanFilter::predictEstimate(Eigen::Matrix<double, 4, 1> extInput)
{
    // calculate priori estimate
    m_estimateApriori = m_A * m_estimateAposteriori + m_B * extInput;

    // calculate priori covariance
    m_covarianceApriori = m_A * m_covarianceAposteriori * (m_A.transpose()) + m_Q;

    return m_estimateApriori;
}

void KalmanFilter::updateEstimate(Eigen::Matrix<double, 2, 1> measurement)
{
    // additional matrix for kalman gain calculation
    Eigen::MatrixXd Sk;
    Sk.resize(r, r);
    Sk = m_R + m_C * m_covarianceApriori * m_C.transpose();
    Sk.inverse();

    // kalman gain
    m_kalmanGain = m_covarianceApriori * (m_C.transpose()) * Sk;

    // error
    m_error = measurement - m_C * m_estimateApriori;

    // calculate posteriori estimate
    m_estimateAposteriori = m_estimateApriori + m_kalmanGain * m_error;

    // steps for posteriori covariance calculation
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(n, n); 
    Eigen::MatrixXd Iminus;
    Iminus.resize(n, n);
    Iminus = I * m_kalmanGain * m_C;

    // calculate posteriori covariance
    m_covarianceAposteriori = Iminus * m_covarianceApriori * (Iminus.transpose()) + m_kalmanGain * m_R * (m_kalmanGain.transpose());
}

void KalmanFilter::resetKalman()
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

void KalmanFilter::update_AQmatrix(double delta_t)
{
    m_A(0, 2) = delta_t;
    m_A(1, 3) = delta_t;

    m_Q(0, 0) = (std::pow(delta_t, 4) / 4) * m_w(0);
    m_Q(0, 2) = (std::pow(delta_t, 3) / 3) * m_w(0);
    m_Q(1, 1) = (std::pow(delta_t, 4) / 4) * m_w(1);
    m_Q(1, 3) = (std::pow(delta_t, 3) / 3) * m_w(1);
    m_Q(2, 0) = (std::pow(delta_t, 3) / 3) * m_w(0);
    m_Q(2, 2) = std::pow(delta_t, 2) * m_w(0);
    m_Q(3, 1) = (std::pow(delta_t, 3) / 3) * m_w(1);
    m_Q(3, 3) = std::pow(delta_t, 2) * m_w(1);
}