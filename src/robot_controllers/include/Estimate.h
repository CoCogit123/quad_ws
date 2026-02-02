#ifndef ESTIMATE_H
#define ESTIMATE_H

#include "Dynamics.h"

#define PROCESS_NOISE_POS 0.01 
#define PROCESS_NOISE_VEL 0.01 
#define PROCESS_NOISE_FOOT 0.01 

#define SENSOR_NOISE_P 0.001
#define SENSOR_NOISE_V 0.1
#define SENSOR_NOISE_Z 0.001

namespace controllers{
    
class Estimate
{
    public:
        Estimate(){}
        void init(Robot_info &robot_data);//初始化 X_k0_update P_k0_update
        void update(Robot_info &robot_data,double dt);
    private:
        Eigen::Vector3d root_pos;  //当前位置（只有世界）
        Eigen::Vector3d root_lin_vel_body;   //当前线速度（机体）
        Eigen::Vector3d root_lin_vel_world;  //当前线速度（世界） 
        
        Eigen::Matrix<double, 18, 1> X_k0_update; //上一时刻后验状态变量 init
        Eigen::Matrix<double, 18, 1> X_k1_est;//当前时刻先验状态变量
        Eigen::Matrix<double, 3, 1> U_k0_update; 
        Eigen::Matrix<double, 18, 18> A;
        Eigen::Matrix<double, 18, 3> B;
        Eigen::Matrix<double, 18, 18> P_k0_update; 
        Eigen::Matrix<double, 18, 18> P_k1_est;
        Eigen::Matrix<double, 18, 18> Q;//预测误差的协方差矩阵

        Eigen::Matrix<double, 28, 1> Z_k0_update; //上一时刻后验
        Eigen::Matrix<double, 28, 1> Z_k1_est;//当前时刻先验
        Eigen::Matrix<double, 28, 18> H;
        Eigen::Matrix<double, 28, 28> R;//观测误差的协方差矩阵

        Eigen::Matrix<double, 28, 28> S;
        Eigen::Matrix<double, 28, 1> error_z; // y-yhat estimated observation
        Eigen::Matrix<double, 28, 1> Serror_z; // S^-1*error_y
        Eigen::Matrix<double, 28, 18> SH; // S^-1*H
        //辅助
        Eigen::Matrix<double, 3, 3> eye3; // 单位矩阵3x3 identity
        double estimated_contacts[4]; //接触程度
};

}

#endif