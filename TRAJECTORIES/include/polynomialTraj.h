#ifndef POLYNOMIAL_TRAJECTORY_H_
#define POLYNOMIAL_TRAJECTROY_H_

#include<vector>
#include<fstream>
#include<string>

// Elementary Trajectories
namespace eleTraj
{
    // linear trajectory
    std::vector<double> linearTraj(double t0,double t1,double q0,double q1, int n);

    // Parabolic trajectory (constant acceleration)
    std::vector<double> parabolicTraj(double t0,double t1, 
        double q0,double q1, double v0, double v1, int n);

}
#endif
