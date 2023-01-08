#ifndef _BASIC_H_
#define _BASIC_H_

#include<vector>
#include<Eigen/Dense>

#define PI 3.1415926
namespace basic
{

    // rotated matrix==========================================================================
    // theta:  rotated angle
    // axis:   x,y,z, or another vector
    // return: Rotated Matrix
    Eigen::MatrixXd rotate(double theta, char axis, Eigen::VectorXd _w = {0,0,0});

    // given R (SO(3))=========================================================================
    // find angle ([0,pi],(-pi,0)), unit rotation axis
    // return a vector: fist element is angle
    //                  last three elements are the axis
    std::vector<double> rRotate(Eigen::Matrix3d R);

    // skew-symmetric==========================================================================
    // x:      vector(1,2,3)
    // return: it's skew-symmetric matrix
    Eigen::MatrixXd skewSymmetric(Eigen::Vector3d x);

    // reverse skew-symmetric==================================================================
    // R:      3x3 skewSymmetric matrix
    // return: vecttor(1,2,3)
    Eigen::VectorXd skewSymmetricReverse(Eigen::Matrix3d R);

    // get the rotation matrix through matrix of exponential so(3)=============================
    // wTheta: [w_theta] = so(3)
    // return: A rotation matrix
    Eigen::MatrixXd matrixExp3(Eigen::Matrix3d wTheta);

    // get the  matrix of exponential so(3) throuth rotation matrix============================
    // R: A rotation matrix
    // return [w_theta] = so(3)
    Eigen::MatrixXd MatrixLog3(Eigen::Matrix3d R);

    // get the transformation matrix===========================================================
    // input DH parameters
    Eigen::MatrixXd trans(double a,double alpha,double d,double theta);

    // inverse of the transformation matrix====================================================
    Eigen::MatrixXd TransInv(Eigen::Matrix4d T);

    //std::vector<double> solveBiQuadratics(std::vector<double> coefficients);

    //std::vector<double> octagonalEqu(const std::vector<double> coefficients);

    // angle calculate=========================================================================
    double angleCalculate(Eigen::Vector3d const S1, Eigen::Vector3d const S2,
                          const Eigen::Vector3d c);

    // get angle through sine and cosine=======================================================
    double angleGet(double sine, double cosine);

    // Returns the se(3) matrix corresponding to a 6-vector twist V.===========================
    // V: a 6-vector twist V.
    Eigen::MatrixXd twistTose3(Eigen::VectorXd V);

    // Returns the 6-vector twist corresponding to an se(3) matrix=============================
    Eigen::VectorXd se3Totwist(Eigen::MatrixXd T);

    // ========================================================================================
    // 6 × 6 adjoint representation [AdT ] of the homogeneous transformation matrix
    Eigen::MatrixXd adJoint(Eigen::MatrixXd T);

    // s: angular velocity
    // q: point on the axis
    // h: pitch
    // return a twist V(w,v)
    Eigen::VectorXd screwTotwist(Eigen::Vector3d q, Eigen::Vector3d s, double h);
    
    // points==================================================================================
    class point
    {
    public:
        point();

        // determine wheater it is point or not
        bool isPoint()
        {
            if(w==0&&x==0&&y==0&&z==0)
            {
                return false;
            }
            return true;
        }

        // three non-parallel planes to formulate a point

    private:
        double x;
        double y;
        double z;
        double w;
    };

    // Plane with unit ========================================================================
    // D0: meters
    // (A,B,C): dimensionless
    class plane
    {
    public:
        plane(double _D0, double _A, double _B, double _C)
        {

        }

        template<class T>
        plane(T S, T r)
        {
            A = S[0];
            B = S[1];
            C = S[2];
            D0 = -(A*r[0]+B*r[1]+C*r[2]);

            if(!isPlane)
            {
                std::cout<<"construct wrong"<<std::endl;
                ~plane();
            }
        }

        // three non-colinear points formulate a plane
        plane(point P, point Q, point R)
        {

        }

        // determine wheather it is plane or not
        bool isPlane()
        {
            if(D0==0&&A==0&&B==0&&C==0)
            {
                return false;
            }
            return true;
        }
        // determine the perpendicular distance from the reference origin to the plane
        double disOrigin()
        {
            double normS = sqrt(A*A+B*B+C*C);
            return -D0/normS;
        }
        
    private:
        double D0;
        double A;
        double B;
        double C;
    };
}
#endif 