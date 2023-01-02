#ifndef _GROUPTHREE_
#define _GROUPTHREE_

#include<vector>
#include<cmath>
#include<Eigen/Dense>

#include "basic.h"
#include "kinematic.h"

namespace iKinematics
{
    // Grpup 3 spatial mechanisims
    // 5R-C // 6R-P 
    class groupThree
    {
    public:
        groupThree() = default;
        groupThree(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta);
        
        ~groupThree()
        {}
    public:
        Eigen::MatrixXd groupThreeMain();

    private:
        std::vector<double> fiveR();

        std::vector<double> sixR();

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

    groupThree::groupThree(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta)
    {

    }
    Eigen::MatrixXd groupThree::groupThreeMain()
    {

    }

    std::vector<double> groupThree::fiveR()
    {

    }

    std::vector<double> groupThree::sixR()
    {
        
    }

}
#endif