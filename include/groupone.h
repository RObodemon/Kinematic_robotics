#ifndef _GROUPONE_
#define _GROUPONE_

#include<vector>
#include<cmath>
#include<Eigen/Dense>

#include "basic.h"
#include "kinematic.h"

namespace iKinematics
{
    // Group 1 spatial mechanisims
    // R-3C // 2R-P-2C // 3R-2P-C // 4R-3P
    class groupOne
    {
    public:
        groupOne() = default;
        groupOne(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta);
        
        ~groupOne()
        {}
    public:
        Eigen::MatrixXd groupOneMain();

    private:
        std::vector<double> oneR(); // 1 situation

        std::vector<double> twoR(); // 18 situations

        std::vector<double> threeR(); // 30 situations

        std::vector<double> fourR(); // 20 situations

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

    groupOne::groupOne(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta)
    {

    }
    Eigen::MatrixXd groupOne::groupOneMain()
    {

    }
    
    std::vector<double> groupOne::oneR()
    {

    }

    std::vector<double> groupOne::twoR()
    {
    
    }

}
#endif