#include <Eigen3/Eigen/Dense>
#include <cmath>

namespace kalman_CP
{
    double h = 0.5;

    Eigen::Vector2d w(0.001, 0.001); 

    Eigen::Matrix<double, 2, 2> A{{1, 0},
                                  {0, 1}};

    Eigen::Matrix<double, 2, 2> B{{h, 0},
                                  {0, h}};

    Eigen::Matrix<double, 2, 2> C{{1, 0},
                                  {0, 1}};

    Eigen::Matrix<double, 2, 1> G{{0}, {0}};

    Eigen::Matrix<double, 2, 2> Q{{std::pow(h, 2) * w(0), 0},
                                  {0, std::pow(h, 2) * w(1)}}; 

    Eigen::Matrix<double, 2, 2> P0{{0.01, 0},
                                   {0, 0.01}};

    Eigen::Matrix<double, 2, 2> R{{0.01, 0},
                                  {0, 0.01}};

    Eigen::Matrix<double, 2, 1> x0{{0.001}, {0.001}};

}

namespace kalman_CV
{
    double h = 0.5;

    Eigen::Vector2d w(0.0001, 0.0001); 

    Eigen::Matrix<double, 4, 4> A{{1, 0, h, 0},
                                  {0, 1, 0, h},
                                  {0, 0, 1, 0},
                                  {0, 0, 0, 1}};

    Eigen::Matrix<double, 4, 4> B{{(std::pow(h, 2) / 2), 0, 0, 0},
                                  {0, (std::pow(h, 2) / 2), 0, 0},
                                  {0, 0, h, 0},
                                  {0, 0, 0, h}};

    // Eigen::Matrix<double, 4, 4> C{{1, 0, 0, 0},
    //                               {0, 1, 0, 0},
    //                               {0, 0, 1, 0},
    //                               {0, 0, 0, 1}};

    Eigen::Matrix<double, 2, 4> C{{1, 0, 0, 0},
                                  {0, 1, 0, 0}};

    Eigen::Matrix<double, 4, 1> G{{0}, {0}, {0}, {0}};

    Eigen::Matrix<double, 4, 4> Q{{(std::pow(h, 4) / 4) * w(0), 0, (std::pow(h, 3) / 2) * w(0), 0},
                                  {0, (std::pow(h, 4) / 4) * w(1), 0, (std::pow(h, 3) / 2) * w(1)},
                                  {(std::pow(h, 3) / 2) * w(0), 0, std::pow(h, 2) * w(0), 0},
                                  {0, (std::pow(h, 3) / 2) * w(1), 0, std::pow(h, 2) * w(1)}}; 

    Eigen::Matrix<double, 4, 4> P0{{0.01, 0, 0, 0},
                                   {0, 0.01, 0, 0},
                                   {0, 0, 0.01, 0},
                                   {0, 0, 0, 0.01}};

    // Eigen::Matrix<double, 4, 4> R{{1, 0, 0, 0},
    //                               {0, 1, 0, 0},
    //                               {0, 0, 1, 0},
    //                               {0, 0, 0, 1}};

    Eigen::Matrix<double, 2, 2> R{{0.5, 0},
                                  {0, 0.5}};

    Eigen::Matrix<double, 4, 1> x0{{0.001}, {0.001}, {0.01}, {0.01}};

}

namespace kalman_CA
{
    double h = 0.5;

    Eigen::Vector2d w(0.0001, 0.0001);

    Eigen::Matrix<double, 6, 6> A{{1, 0, h, 0, (std::pow(h, 2) / 2), 0},
                                  {0, 1, 0, h, 0, (std::pow(h, 2) / 2)},
                                  {0, 0, 1, 0, h, 0},
                                  {0, 0, 0, 1, 0, h},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1}};

    Eigen::Matrix<double, 6, 6> B{{(std::pow(h, 3) / 3), 0, 0, 0, 0, 0},
                                  {0, (std::pow(h, 3) / 3), 0, 0, 0, 0},
                                  {0, 0, (std::pow(h, 2) / 2), 0, 0, 0},
                                  {0, 0, 0, (std::pow(h, 2) / 2), 0, 0},
                                  {0, 0, 0, 0, h, 0},
                                  {0, 0, 0, 0, 0, h}};

    Eigen::Matrix<double, 6, 6> C{{1, 0, 0, 0, 0, 0},
                                  {0, 1, 0, 0, 0, 0},
                                  {0, 0, 1, 0, 0, 0},
                                  {0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1}};

    Eigen::Matrix<double, 6, 1> G{{0}, {0}, {0}, {0}, {0}, {0}};

    Eigen::Matrix<double, 6, 6> Q{{(std::pow(h, 6) / 9) * w(0), 0, (std::pow(h, 5) / 6) * w(0), 0, (std::pow(h, 4) / 3) * w(0), 0},
                                  {0, (std::pow(h, 6) / 9) * w(1), 0, (std::pow(h, 5) / 6) * w(1), 0, (std::pow(h, 4) / 3) * w(1)},
                                  {(std::pow(h, 5) / 6) * w(0), 0, (std::pow(h, 4) / 4) * w(0), 0, (std::pow(h, 3) / 2) * w(0), 0},
                                  {0, (std::pow(h, 5) / 6) * w(1), 0, (std::pow(h, 4) / 4) * w(1), 0, (std::pow(h, 3) / 2) * w(1)},
                                  {(std::pow(h, 4) / 3) * w(0), 0, (std::pow(h, 3) / 2) * w(0), 0, std::pow(h, 2) * w(0), 0},
                                  {0, (std::pow(h, 4) / 3) * w(1), 0, (std::pow(h, 3) / 2) * w(1), 0, std::pow(h, 2) * w(1)}};

    Eigen::Matrix<double, 6, 6> P0{{0.05, 0, 0, 0, 0, 0},
                                   {0, 0.05, 0, 0, 0, 0},
                                   {0, 0, 0.05, 0, 0, 0},
                                   {0, 0, 0, 0.05, 0, 0},
                                   {0, 0, 0, 0, 0.05, 0},
                                   {0, 0, 0, 0, 0, 0.05}};

    Eigen::Matrix<double, 6, 6> R{{0.01, 0, 0, 0, 0, 0},
                                  {0, 0.01, 0, 0, 0, 0},
                                  {0, 0, 0.01, 0, 0, 0},
                                  {0, 0, 0, 0.01, 0, 0},
                                  {0, 0, 0, 0, 0.01, 0},
                                  {0, 0, 0, 0, 0, 0.01}};

    Eigen::Matrix<double, 6, 1> x0{{0}, {0}, {0}, {0}, {0.01}, {0.01}};

}