#include<vector>
#include<cmath>
#include<Eigen/Dense>

namespace kine
{
    // Help to get the parameters which are used to help do the reverse kinematics
    // output parameters:
    // A71, S7, S1, alpha71, theta7, gamma1
    std::vector<double> closeLoopAnalysis(Eigen::Vector3d PtoolTo6, Eigen::Vector3d PtoolToF,
        Eigen::Vector3d S6ToF, Eigen::Vector3d a67ToF );

}
