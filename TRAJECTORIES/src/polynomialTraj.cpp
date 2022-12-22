#ifndef POLYNOMIAL_TRAJECTORY_H_
#define POLYNOMIAL_TRAJECTROY_H_

#include<iostream>
#include<vector>
#include<fstream>
#include<string>

#include "polynomialTraj.h"

// Elementary Trajectories
namespace eleTraj
{
    // linear trajectory
    std::vector<double> linearTraj(double t0,double t1,double q0,double q1, int n)
    {
        double a0 = q0;
        double a1 = (q1-q0)/(t1-t0);
        std::vector<double> traj(n+2,0);
        double temp = (t1-t0)/n;
        
        std::ofstream fout("C:/Users/Henyue Guan/Desktop/DSC/Robot/Robotics Arm/Project/Trajectories/data_plot/linear_trj.txt",std::ios::out);
        if (fout.is_open() == false)
        {
            std::cout << "open file" << " " << "fails\n";
        }

        for(int i=0;i<n;i++)
        {
            if(i==12)
            {
                std::cout<<"input data"<<std::endl;
            }
            double t = t0+temp*i;
            traj[i] = a0+a1*t;
            fout<<t<<" "<<traj[i]<<std::endl;
        }
        traj[n] = a1;
        traj[n+1] = 0;
        fout.close();
        return traj;
    }

    // Parabolic trajectory (constant acceleration)
    std::vector<double> parabolicTraj(double t0,double t1, 
        double q0,double q1, double v0, double v1, int n)
    {
        double a0 = q0;
        double a1 = v0;
        double T = t1-t0;
        double h = 0;
        if(q1>=q0)
        {
            h = q1-q0;
        }
        else
        {
            h = q0-q1;
        }
        double a2 = 2.0*(h-v0*T)/T*T;

        double a3 = (q0+q1)/2;
        double a4 = 2*h/T-v1;
        double a5 = 2*(v1*T-h)/(T*T);

        std::vector<double> traj(2*n+1,0);
        double temp = T/n;
        std::string filename = R"(data_plot\parabolic_Trj.txt)";
        std::ofstream fout;
        for(int i=0;i<n/2;i++)
        {
            double t = temp*i;
            traj[i] = a0+a1*t+a2*t*t;
            traj[i+n] = a1+2*a2*t;
        }
        for(int i = n/2;i<n;i++)
        {
            double t = temp*t;
            traj[i] =  a3+a4*t+a5*t*t;
            traj[i+n] = a4+2*a5*t;
        }
        traj[2*n] = 4*(h-v0*T)/T/T;
        fout.close();

        return traj;
    }


}
#endif
