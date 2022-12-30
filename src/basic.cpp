#include"basic.h"

#include<cmath>
#include<math.h>
#include<iostream>

namespace basic
{
    Eigen::MatrixXd rotate(double theta, char axis, Eigen::VectorXd _w = {0,0,0})
    {
        Eigen::VectorXd w = w;

        if(axis=='x'||axis=='X')
        {
            w<<1,0,0;
        }
        else if(axis=='y'||axis=='Y')
        {
            w<<0,1,0;
        }
        else if(axis=='z'||axis=='Z')
        {
            w<<0,0,1;
        }
        double w1 = w(1);
        double w2 = w(2);
        double w3 = w(3);

        Eigen::MatrixXd Rot;
        double v = 1-cos(theta);
        Rot<<cos(theta)+w1*w1*v,    w1*w2*v-w3*sin(theta), w1*w3*v+w2*sin(theta),
             w1*w2*v+w3*sin(theta), cos(theta)+w2*w2*v,    w2*w3*v-w1*sin(theta),
             w1*w3*v-w2*sin(theta), w2*w3*v+w1*sin(theta), cos(theta)+w3*w3*v;

        return Rot;
    }

    std::vector<double> rRotate(Eigen::MatrixXd R)
    {
        std::vector<double> result(4,0);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        if(R==I)
        {
            std::cout<<" Axis Undefined "<<std::endl;
            return result;
        }
        if((R(0,0)+R(1,1)+R(2,2))==-1)
        {
            result[0] = PI;
            double temp = 1/sqrt(1*(1+R(2,2)));
            result[1] = temp*R(0,2);
            result[2] = temp*R(1,2);
            result[3] = temp*(1+R(2,2));

            return result;
        }

        result[0] = acos(1/2*(R(0,0)+R(1,1)+R(2,2)-1));
        Eigen::MatrixXd Rt = R.transpose();
        double temp = 1/sin(result[0]);
        result[1] = temp*(R(2,1)-R(1,2));
        result[2] = temp*(R(0,2)-R(2,0));
        result[3] = temp*(R(1,0)-R(0,1));

        return result;
    }

    double angleCalculate(const Eigen::Vector3d a, const Eigen::Vector3d b, 
        const Eigen::Vector3d c)
    {
        Eigen::Vector3d S1 = a/a.norm();
        Eigen::Vector3d S2 = b/b.norm();
        Eigen::Vector3d a12 = c/c.norm();

        // calculate cosine and sine
        double c12 = S1.dot(S2);
        Eigen::Vector3d temp = S1.cross(S2);
        double s12 = temp.dot(a12);

        double angle = 0.0;

        if((s12*s12+c12*c12)!=1)
        {
            std::cout<<"wrong input "<<std::endl;
        }
        if(c12>=0)
        {
            angle =  asin(s12);
        }
        else
        {
            if(s12>=0)
            {
                angle = acos(c12);
            }
            else
            {
                angle = -acos(c12);
            }
        }

        return angle;
    }

    double trigonometric(double A, double B, double D)
    {
        if(A==0&&B==0)
        {
            std::cout<<"wrong input";
            return 0;
        }

        
    }
    // std::vector<double> octagonalEqu(const std::vector<double> coefficients)
    // {
    //     double a1,b1,d1,e1,f1,g1,h1,i1,j1,a2,b2,d2,e2,f2,g2,h2,i2,j2;
    //     a1 = coefficients[0]; a2 = coefficients[9];
    //     b1 = coefficients[1]; b2 = coefficients[10];
    //     d1 = coefficients[2]; d2 = coefficients[11];
    //     e1 = coefficients[3]; e2 = coefficients[12];
    //     f1 = coefficients[4]; f2 = coefficients[13];
    //     g1 = coefficients[5]; g2 = coefficients[14];
    //     h1 = coefficients[6]; h2 = coefficients[15];
    //     i1 = coefficients[7]; i2 = coefficients[16];
    //     j1 = coefficients[8]; j2 = coefficients[17];
        

    //     double x1,L1,L2,M1,M2,N1,N2;
    //     double x1Square = x1*x1;
    //     L1 = a1*x1Square+b1*x1+d1;
    //     L2 = a2*x1Square+b2*x1+d2;
    //     M1 = e1*x1Square+f1*x1+g1;
    //     M2 = e2*x1Square+f2*x1+g2;
    //     N1 = h1*x1Square+i1*x1+j1;
    //     N2 = h2*x1Square+i2*x1+j2;
    //     // I need to get the coefficients
    //     double Y = (L1*M2-M1*L2)*(M1*N2-N1*M2)-(L1*N2-N1*L2)*(L1*N2-N1*L2);
    // }
    // std::vector<double> solveBiQuadratics(std::vector<double> coefficients)
    // {
    //     double a1,b1,d1,e1,f1,g1,h1,i1,j1,a2,b2,d2,e2,f2,g2,h2,i2,j2;
    //     a1 = coefficients[0]; a2 = coefficients[9];
    //     b1 = coefficients[1]; b2 = coefficients[10];
    //     d1 = coefficients[2]; d2 = coefficients[11];
    //     e1 = coefficients[3]; e2 = coefficients[12];
    //     f1 = coefficients[4]; f2 = coefficients[13];
    //     g1 = coefficients[5]; g2 = coefficients[14];
    //     h1 = coefficients[6]; h2 = coefficients[15];
    //     i1 = coefficients[7]; i2 = coefficients[16];
    //     j1 = coefficients[8]; j2 = coefficients[17];

    //     double L1,L2,M1,M2,N1,N2;
    //     double x1Square = 0;
    //     std::vector<double> x1 = octagonalEqu(coefficients);
    //     std::vector<double> x2(10,0);
    //     for(int i=0;i<8;i++)
    //     {
    //         x1Square = x1[i]*x1[i];
    //         L1 = a1*x1Square+b1*x1[i]+d1;
    //         L2 = a2*x1Square+b2*x1[i]+d2;
    //         M1 = e1*x1Square+f1*x1[i]+g1;
    //         M2 = e2*x1Square+f2*x1[i]+g2;
    //         N1 = h1*x1Square+i1*x1[i]+j1;
    //         N2 = h2*x1Square+i2*x1[i]+j2;
    //         x2[i] = -(M1*N2-N1*M2)/(L1*N2-N1*L2);
    //     }
    //     return x2;
    // }
}











std::vector<double> octagonalEqu(const std::vector<double> coefficients)
{
    double a1,b1,d1,e1,f1,g1,h1,i1,j1,a2,b2,d2,e2,f2,g2,h2,i2,j2;
    a1 = coefficients[0]; a2 = coefficients[9];
    b1 = coefficients[1]; b2 = coefficients[10];
    d1 = coefficients[2]; d2 = coefficients[11];
    e1 = coefficients[3]; e2 = coefficients[12];
    f1 = coefficients[4]; f2 = coefficients[13];
    g1 = coefficients[5]; g2 = coefficients[14];
    h1 = coefficients[6]; h2 = coefficients[15];
    i1 = coefficients[7]; i2 = coefficients[16];
    j1 = coefficients[8]; j2 = coefficients[17];
    

    double x1,L1,L2,M1,M2,N1,N2;
    double x1Square = x1*x1;
    L1 = a1*x1Square+b1*x1+d1;
    L2 = a2*x1Square+b2*x1+d2;
    M1 = e1*x1Square+f1*x1+g1;
    M2 = e2*x1Square+f2*x1+g2;
    N1 = h1*x1Square+i1*x1+j1;
    N2 = h2*x1Square+i2*x1+j2;
}


std::vector<double> solveBiQuadratics(std::vector<double> coefficients)
{
    double a1,b1,d1,e1,f1,g1,h1,i1,j1,a2,b2,d2,e2,f2,g2,h2,i2,j2;
    a1 = coefficients[0]; a2 = coefficients[9];
    b1 = coefficients[1]; b2 = coefficients[10];
    d1 = coefficients[2]; d2 = coefficients[11];
    e1 = coefficients[3]; e2 = coefficients[12];
    f1 = coefficients[4]; f2 = coefficients[13];
    g1 = coefficients[5]; g2 = coefficients[14];
    h1 = coefficients[6]; h2 = coefficients[15];
    i1 = coefficients[7]; i2 = coefficients[16];
    j1 = coefficients[8]; j2 = coefficients[17];

    double L1,L2,M1,M2,N1,N2;
    double x1Square = 0;
    std::vector<double> x1 = octagonalEqu(coefficients);
    std::vector<double> x2(10,0);
    for(int i=0;i<8;i++)
    {
        x1Square = x1[i]*x1[i];
        L1 = a1*x1Square+b1*x1[i]+d1;
        L2 = a2*x1Square+b2*x1[i]+d2;
        M1 = e1*x1Square+f1*x1[i]+g1;
        M2 = e2*x1Square+f2*x1[i]+g2;
        N1 = h1*x1Square+i1*x1[i]+j1;
        N2 = h2*x1Square+i2*x1[i]+j2;
        x2[i] = -(M1*N2-N1*M2)/(L1*N2-N1*L2);
    }
    return x2;
}