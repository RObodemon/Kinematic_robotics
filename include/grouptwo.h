#ifndef _GROUPTWO_
#define _GROUPTWO_

#include<vector>
#include<cmath>
#include<Eigen/Dense>

#include "basic.h"
#include "kinematic.h"

namespace iKinematics
{
    /// Grpup 2 spatial mechanisims
    // 3R-2C // 4R-P-C // 5R-2P
    class groupTwo
    {
    public:
        groupTwo() = default;
        groupTwo(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta);
        
        ~groupTwo()
        {}
    public:
        Eigen::MatrixXd groupTwoMain();

    private:
        std::vector<double> threeR(); 

        std::vector<double> fourR(); 

        std::vector<double> fiveR();

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

    groupTwo::groupTwo(std::vector<double> _a, std::vector<double> _alpha, 
                 std::vector<double> _S, std::vector<int> unknownS,
                 std::vector<double> _theta, std::vector<int> unkonwnTheta)
    {

    }
    Eigen::MatrixXd groupTwo::groupTwoMain()
    {

    }
    
    std::vector<double> groupTwo::threeR()
    {

    }

    std::vector<double> groupTwo::fourR()
    {
    
    }

    std::vector<double> groupTwo::fiveR()
    {

    }

}
#endif