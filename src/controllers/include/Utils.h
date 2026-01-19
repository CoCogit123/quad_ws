#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace controllers{

class utils
{
    public:
        utils(){}
        static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
        static Eigen::Matrix3d euler_to_rot(Eigen::Vector3d euler);
        
        static Eigen::Vector3d bezier_pos(Eigen::Vector3d start,Eigen::Vector3d end,double swing_high,double devel);
        static Eigen::Vector3d bezier_vel(Eigen::Vector3d start,Eigen::Vector3d end,double swing_high,double devel);

        static Eigen::Matrix3d skew(Eigen::Vector3d vec);
                            
};

}


#endif