#ifndef _GROUPFOUR_
#define _GROUPFOUR_

#include<vector>
#include<cmath>
#include<Eigen/Dense>

#include "basic.h"
#include "kinematic.h"

namespace iKinematics
{
    // Group 4 spatial mechanisims
    // 7R
    class groupFour
    {
    public:
        groupFour() = default;
        groupFour(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta);
        
        ~groupFour()
        {}
    public:
        Eigen::MatrixXd groupFourMain();

    private:
        std::vector<double> sevenR();

    private:
        // given parameters
        std::vector<double> a;
        std::vector<double> alpha;
        std::vector<double> S;
        std::vector<double> theta;
        // find parameters
        std::vector<double> SFind;
        std::vector<double> thetaFind;
    };

    groupFour::groupFour(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta)
    {

    }
    Eigen::MatrixXd groupFour::groupFourMain()
    {

    }

    std::vector<double> groupFour::sevenR()
    {

    }
}
#endif