
#include<cmath>
#include<math.h>
#include<iostream>

#include"basic.h"

namespace basic
{
    // rotated matrix==========================================================================
    // theta: rotated angle
    // axis: x,y,z, or another vector
    // return: Rotated Matrix
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

    // given R (SO(3))=========================================================================
    // find angle ([0,pi],(-pi,0)), unit rotation axis
    // return a vector: fist element is angle
    //                  last three elements are the axis
    std::vector<double> rRotate(Eigen::Matrix3d R)
    {
        std::vector<double> result(4,0);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        if(R==I)
        {
            std::cout<<" Axis Undefined, theta = 0"<<std::endl;
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
        double temp = 1/(2*sin(result[0]));
        result[1] = temp*(R(2,1)-R(1,2));
        result[2] = temp*(R(0,2)-R(2,0));
        result[3] = temp*(R(1,0)-R(0,1));

        return result;
    }

    // skew-symmetric==========================================================================
    // x: vector(1,2,3)
    // return it's skew-symmetric matrix
    Eigen::MatrixXd skewSymmetric(Eigen::Vector3d x)
    {
        Eigen::MatrixXd result(3,3);
        result<<    0, -x[2],  x[1], 
                 x[2],     0, -x[0],
                -x[1],  x[0],     0;

        return result;
    }
    // reverse skew-symmetric==================================================================
    // R:      3x3 skewSymmetric matrix
    // return: vecttor(1,2,3)
    Eigen::VectorXd skewSymmetricReverse(Eigen::Matrix3d R)
    {
        Eigen::VectorXd result(3);
        result <<R(2,1), -R(2,0),R(1,0);

        return result;
    }

    // get the rotation matrix through matrix of exponential so(3)=============================
    // wTheta: [w_theta] = so(3) (skew-symmetrc matrix)
    // return: A rotation matrix
    Eigen::MatrixXd matrixExp3(Eigen::Matrix3d wTheta)
    {
        // get the rotate axis which is not an unit vector
        Eigen::VectorXd wHat = skewSymmetricReverse(wTheta);
        // its magnitude it the theta
        double theta = wHat.norm();
        Eigen::MatrixXd w = wTheta/theta;

        Eigen::MatrixXd R(3,3);
        R = Eigen::MatrixXd::Identity(3,3) + sin(theta)*w + (1-cos(theta))*w*w;

        return R;
    }

    // get the  matrix of exponential so(3) throuth rotation matrix============================
    // R: A rotation matrix
    // return [w_theta] = so(3)
    Eigen::MatrixXd MatrixLog3(Eigen::Matrix3d R)
    {
        std::vector<double> angleAxis = rRotate(R);
        Eigen::MatrixXd w_theta;
        w_theta<<           0, -angleAxis[3],  angleAxis[2], 
                 angleAxis[3],             0, -angleAxis[1],
                -angleAxis[2],  angleAxis[1],             0;

        w_theta = w_theta*angleAxis[0];

        return w_theta;
    }

    // get the transformation matrix===========================================================
    // input DH parameters
    Eigen::MatrixXd trans(double a,double alpha,double d,double theta)
    {
        Eigen::MatrixXd Talpha;
        Eigen::MatrixXd Ta;
        Eigen::MatrixXd Ttheta;
        Eigen::MatrixXd Td;

        Talpha << 1,          0,           0,  0,
                  0, cos(alpha), -sin(alpha),  0,
                  0, sin(alpha),  cos(alpha),  0,
                  0,          0,           0,  1;

        Ta << 1, 0, 0, a,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        Ttheta << cos(theta), -sin(theta), 0, 0,
                  sin(theta),  cos(theta), 0, 0,
                           0,           0, 1, 0,
                           0,           0, 0, 1;
        
        Td << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, d,
              0, 0, 0, 1;

        return Talpha*Ta*Ttheta*Td;
    }

    // inverse of the transformation matrix====================================================
    Eigen::MatrixXd TransInv(Eigen::Matrix4d T)
    {
        // dynamic size
        Eigen::Matrix3d R = T.block(0,0,3,3);
        // fixed size
        // Eigen::Matrix3d R = T.block<3,3>(0,0);

        Eigen::Vector3d P = T.block(0,3,4,1);

        Eigen::Matrix3d Rtemp = R.transpose();
        Eigen::Vector3d Ptemp = -R*P;
        Eigen::MatrixXd temp;
        temp.resize(3,3);
        temp<<Rtemp,Ptemp;

        Eigen::MatrixXd result;
        result.resize(4,4);
        result<<temp,
                0,0,0,1;

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

    double angleGet(double sine, double cosine)
    {
        double angle = 0;
        if(cosine>=0)
        {
            angle =  asin(sine);
        }
        else
        {
            if(sine>=0)
            {
                angle = acos(cosine);
            }
            else
            {
                angle = -acos(cosine);
            }
        }
    }

    std::vector<double> trigonometric(double A, double B, double D)
    {
        if(A==0&&B==0)
        {
            std::cout<<"wrong input";
            return;
        }

        std::vector<double> result(2,PI);
        if(A==0&&D==0&&B!=0)
        {
            result[0] = 0;
            result[1] = 1;
            return result;
        }
        if(A==D&&A!=0&&B==0)
        {
            return result;
        }

        double sinGamma = B/sqrt(A*A+B*B);
        double cosGamma = A/sqrt(A*A+B*B);
        double gamma = angleGet(sinGamma,cosGamma);

        result[0] = gamma + acos(-D/sqrt(A*A+B*B));
        result[1] = gamma - acos(-D/sqrt(A*A+B*B));
        return result;
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