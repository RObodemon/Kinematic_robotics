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
        
        std::ofstream fout("your location/data_plot/linear_trj.txt",std::ios::out);
        if (fout.is_open() == false)
        {
            std::cout << "open file" << " " << "fails\n";
        }

        for(int i=0;i<=n;i++)
        {
            if(i==12)
            {
                std::cout<<"input data"<<std::endl;
            }
            double t = t0+temp*i;
            traj[i] = a0+a1*t;
            fout<<t<<" "<<traj[i]<<" "<<a1<<" "<<0<<std::endl;
        }
        traj[n] = a1;
        traj[n+1] = 0;
        fout.close();
        return traj;
    }

    // Parabolic trajectory (constant acceleration)
    std::vector<double> parabolicTraj(double t0,double t1, double tf,
        double q0,double q1, double v0, double v1, int n)
    {
        double T  = t1-t0;
        double Ta = tf-t0; 
        double Td = t1-tf;

        double a0 = q0;
        double a1 = v0;
        double h  = 0;
        if(q1>=q0)
            h = q1-q0;
        else
            h = q0-q1;

        double a2 = (2.0*h-v0*(T+Ta)-v1*Td)/(2*T*Ta);
        double a3 = (2.0*q1*Ta+Td*(2*q0+Ta*(v0-v1)))/(2*T);
        double a4 = (2.0*h-v0*Ta-v1*Td)/T;
        double a5 = -(2.0*h-v0*Ta-v1*(T+Td))/(2*T*Td);

        std::vector<double> traj(2*n+2,0);
        double temp = T/n;
        std::ofstream fout("your location/data_plot/Parabolic_trj.txt",std::ios::out);
        if (fout.is_open() == false)
        {
            std::cout << "open file" << " " << "fails\n";
        }

        traj[2*n] = 2*a2; // acceleration
        int sec = n*Ta/T;
        for(int i=0;i<sec;i++)
        {
            double t = temp*i+t0;
            traj[i] = a0+a1*(t-t0)+a2*(t-t0)*(t-t0);  // position
            traj[i+n] = a1+2*a2*(t-t0);     // velocity
            fout<<t<<" "<<traj[i]<<" "<<traj[i+n]<<" "<<traj[2*n]<<std::endl; // the forth column is acceleration, which is a constant
        }

        traj[2*n+1] = 2*a5;  // acceleration
        for(int i = sec;i<=n;i++)
        {
            double t = temp*i+t0;
            traj[i] =  a3+a4*(t-tf)+a5*(t-tf)*(t-tf);  // postion
            traj[i+n] = a4+2*a5*(t-tf);      // velocity
            fout<<t<<" "<<traj[i]<<" "<<traj[i+n]<<" "<<traj[2*n+1]<<std::endl;  // the forth column is acceleration, which is a constant
        }

        fout.close();
        return traj;
    }

}
#endif
