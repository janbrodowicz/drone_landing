#include "Regulators/KalmanFilter.h"


using namespace kalman;


KalmanFilter::KalmanFilter(Eigen::MatrixXd A_in, Eigen::MatrixXd B_in, Eigen::MatrixXd C_in,
                           Eigen::MatrixXd Q_in, Eigen::MatrixXd R_in, Eigen::MatrixXd P0_in,
                           Eigen::MatrixXd x0_in)
{
    // initialize matrices
    A = A_in; B = B_in; C = C_in; Q = Q_in;
    R = R_in; P0 = P0_in; x0 = x0_in;

    // extract the appropriate dimensions
    n = A.rows();   m = B.cols(); r = C.rows();

    // initial guess
    estimateAposteriori = x0;

    // initial estimate covariance
    covarianceAposteriori = P0;

    // set sizes of the rest of matrices and set with zeros
    estimateApriori.resize(n, 1);
    estimateApriori.setZero();

    covarianceApriori.resize(n, n);
    covarianceApriori.setZero();

    kalmanGain.resize(n, r);
    kalmanGain.setZero();

    error.resize(r, 1);
    error.setZero();
}

KalmanFilter::KalmanFilter(const KalmanFilter& filter)
{
    // initialize matrices
    A = filter.A; B = filter.B; C = filter.C; Q = filter.Q;
    R = filter.R; P0 = filter.P0; x0 = filter.x0;

    // extract the appropriate dimensions
    n = A.rows();   m = B.cols(); r = C.rows();

    // initial guess
    estimateAposteriori = x0;

    // initial estimate covariance
    covarianceAposteriori = P0;

    // set sizes of the rest of matrices and set with zeros
    estimateApriori.resize(n, 1);
    estimateApriori.setZero();

    covarianceApriori.resize(n, n);
    covarianceApriori.setZero();

    kalmanGain.resize(n, r);
    kalmanGain.setZero();

    error.resize(r, 1);
    error.setZero();
}

void KalmanFilter::predictEstimate(Eigen::MatrixXd extInput, Eigen::MatrixXd& estimateApriori, Eigen::MatrixXd& covarianceApriori)
{
    // calculate priori estimate
    estimateApriori = A * estimateAposteriori + B * extInput;

    // calculate priori covariance
    covarianceApriori = A * covarianceAposteriori * (A.transpose()) + Q;
}

void KalmanFilter::updateEstimate(Eigen::MatrixXd measurement, Eigen::MatrixXd& estimateAposteriori, Eigen::MatrixXd& covarianceAposteriori)
{
    // additional matrix for kalman gain calculation
    Eigen::MatrixXd Sk;
    Sk.resize(r, r);
    Sk = R + C * covarianceApriori * C.transpose();
    Sk.inverse();

    // kalman gain
    kalmanGain = covarianceApriori * (C.transpose()) * Sk;

    // error
    error = measurement - C * estimateApriori;

    // calculate posteriori estimate
    estimateAposteriori = estimateApriori + kalmanGain * error;

    // steps for posteriori covariance calculation
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(n, n); 
    Eigen::MatrixXd Iminus;
    Iminus.resize(n, n);
    Iminus = I * kalmanGain * C;

    // calculate posteriori covariance
    covarianceAposteriori = Iminus * covarianceApriori * (Iminus.transpose()) + kalmanGain * R * (kalmanGain.transpose());
}

void KalmanFilter::resetKalman()
{
    // reset posteriori estimate matrix with initial guess
    estimateAposteriori = x0;

    // reset posteriori matrix with initial covariance
    covarianceAposteriori = P0;

    // reseting matrices
    estimateApriori.setZero();

    covarianceApriori.setZero();

    kalmanGain.setZero();

    error.setZero();
}

void KalmanFilter::update_Amatrix(double delta_t)
{
    A(0, 2) = delta_t;
    A(1, 3) = delta_t;
}