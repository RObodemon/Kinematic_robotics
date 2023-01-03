#include<vector>
#include<cmath>
#include<Eigen/Dense>

namespace iKinematics
{
    // Help to get the parameters which are used to help do the reverse kinematics
    // output parameters:
    // A71, S7, S1, alpha71, theta7, gamma1
    std::vector<double> closeLoop(Eigen::Vector3d PtoolTo6, Eigen::Vector3d PtoolToF,
        Eigen::Vector3d S6ToF, Eigen::Vector3d a67ToF );


    // Given the robotics parameters can help calculate the reverse kinematics
    std::vector<double> reverseMain();
    
    // Derivation of fundamental sine, sine-cosine, and cosine laws
    double X(std::vector<int> num, std::vector<double> alpha, std::vector<double> theta);

    double Xstar(std::vector<int> num, std::vector<double> alpha, std::vector<double> theta);

    double Y(std::vector<int> num, std::vector<double> alpha, std::vector<double> theta);

    double Z(std::vector<int> num, std::vector<double> alpha, std::vector<double> theta);
}