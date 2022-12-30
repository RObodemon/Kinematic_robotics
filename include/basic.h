#ifndef _BASIC_H_
#define _BASIC_H_

#include<vector>
#include<Eigen/Dense>

#define PI 3.1415926
namespace basic
{
    //std::vector<double> solveBiQuadratics(std::vector<double> coefficients);
    //std::vector<double> octagonalEqu(const std::vector<double> coefficients);

    // rotated matrix
    // axis: x,y,z, or another vector
    // return: Rotated Matrix
    Eigen::MatrixXd rotate(double theta, char axis, Eigen::VectorXd _w = {0,0,0});

    // given R (SO(3))
    // find angle ([0,pi]), unit rotation axis
    // return a vector: fist element is angle
    //                  last three elements are the axis
    std::vector<double> rRotate(Eigen::MatrixXd R);
    
    // Function: 获得绕某个轴旋转一定角度的旋转矩阵
    // Inputs:   输入旋转轴的向量，旋转角度
    // Outputs:  输出旋转矩阵

    // Function: 获得旋转轴的单位矩阵
    // Inputs:   输入旋转矩阵
    // Outputs:  输出旋转轴的单位向量

    // 获得 skew-symetric matrix


    // angle calculate
    double angleCalculate(Eigen::Vector3d const S1, Eigen::Vector3d const S2,const Eigen::Vector3d c);

    // get angle through sine and cosine
    double angleGet();
    // Trigonometric Solution of A*c1 + B*s1 + D = 0
    std::vector<double> trigonometric(double A, double B, double D);
}
#endif 