#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include "Common.h"

namespace controllers{

class utils
{
    public:
        utils(){}
        static Vector3d quat_to_euler(Eigen::Quaterniond quat);
        static Matrix3d euler_to_rot(Vector3d euler);
        
        static Vector3d bezier_pos(Vector3d start,Vector3d end,double swing_high,double devel);
        static Vector3d bezier_vel(Vector3d start,Vector3d end,double swing_high,double devel);

        static Matrix3d skew(Vector3d vec);

        static void WeightedInverse(const Eigen::MatrixXd& J, const Eigen::MatrixXd& Winv, 
                                Eigen::MatrixXd& Jinv, double threshold = 0.0001);
        static void PseudoInverse(const Eigen::MatrixXd& mat, double threshold, Eigen::MatrixXd& inv);
                            
};

}


#endif