#include"kinematic.h"
#include "basic.h"
#define PI 3.1415926
namespace kine
{
    std::vector<double> closeLoopAnalysis(Eigen::Vector3d PtoolTo6, Eigen::Vector3d PtoolToF,
        Eigen::Vector3d s6ToF, Eigen::Vector3d a67ToF )
    {
        Eigen::Vector3d xi,yj,zk;
        xi<<1,0,0;
        yj<<0,1,0;
        zk<<0,0,1;

        Eigen::Vector3d P6ToF = PtoolToF-(PtoolTo6.dot(xi))*a67ToF
            -((PtoolTo6.dot(yj))*s6ToF).cross(a67ToF)
            -(PtoolTo6.dot(zk))*s6ToF;
        
        Eigen::Vector3d s7ToF = a67ToF.cross(s6ToF);
        Eigen::Vector3d s1ToF;
        s1ToF<< 0,0,1;

        double c71 = s7ToF.dot(s1ToF);
        // Parallel
        if(c71==1||c71==-1)
        {
            double alpha71 = PI;
            if(c71==0)
                alpha71 = 0;

            double S7 = 0;
            double S1 = -P6ToF.dot(s1ToF);
            double A71 = (-P6ToF-s1ToF*S1).norm();

            double alpha71 = PI;
                if(c71==0)
                    alpha71 = 0;
            if(A71==0)
            {
                // colinear
                double theta7 = 0;
                double gamma1 = basic::angleCalculate(a67ToF,xi,s1ToF);
                std::vector<double> param = {A71, S7, S1, alpha71, theta7, gamma1};
                return param;
            }
            
            Eigen::Vector3d a71ToF = -(P6ToF+s1ToF*S1)/A71;
            double theta7 = basic::angleCalculate(a67ToF,a71ToF,s7ToF);
            double gamma1 = basic::angleCalculate(a71ToF,xi,s1ToF);
            
            std::vector<double> param = {A71, S7, S1, alpha71, theta7, gamma1};
            return param;
        }

        Eigen::Vector3d a71ToF = s7ToF.cross(s1ToF);
        a71ToF = a71ToF/a71ToF.norm();
        double alpha71 = basic::angleCalculate(s7ToF,s1ToF,a71ToF);
        double theta7 = basic::angleCalculate(a67ToF,a71ToF,s7ToF);

        double c71 = cos(alpha71);
        
        double gamma1 = basic::angleCalculate(a71ToF,xi,s1ToF);
        double S7 = (s1ToF.cross(P6ToF)).dot(a71ToF)/sin(alpha71);
        double A71 = (P6ToF.cross(s1ToF)).dot(s7ToF)/sin(alpha71);
        double S1 = (P6ToF.cross(s7ToF)).dot(a71ToF)/sin(alpha71);
        
        // A71, S7, S1, alpha71, theta7, gamma1
        std::vector<double> param = {A71, S7, S1, alpha71, theta7, gamma1};
        return param;
    }
}
