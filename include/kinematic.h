#include<vector>
#include<cmath>
#include<Eigen/Dense>

namespace rKinematics
{
    // Help to get the parameters which are used to help do the reverse kinematics
    // output parameters:
    // A71, S7, S1, alpha71, theta7, gamma1
    std::vector<double> closeLoop(Eigen::Vector3d PtoolTo6, Eigen::Vector3d PtoolToF,
        Eigen::Vector3d S6ToF, Eigen::Vector3d a67ToF );


    // Given the robotics parameters can help calculate the reverse kinematics
    std::vector<double> reverseMain();
    
    // Group 1 spatial mechanisims
    // R-3C // 2R-P-2C // 3R-2P-C // 4R-3P

    std::vector<double> twoR();

    std::vector<double> oneR();

    std::vector<double> threeR();

    std::vector<double> fourR();

    // Grpup 2 spatial mechanisims
    // 3R-2C // 4R-P-C // 5R-2P


    // Grpup 3 spatial mechanisims
    // 5R-C // 6R-P 
    
    // Group 4 spatial mechanisims
    // 7R
}