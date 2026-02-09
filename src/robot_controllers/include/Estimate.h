#ifndef ESTIMATE_H
#define ESTIMATE_H

#include "Common.h"

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
        void update(Robot_info &robot_data,Gait_info& gait_data,double dt);

        bool init_flag = false;
    private:


        Eigen::Vector3d root_pos;  //当前位置（只有世界）
        Eigen::Vector3d root_lin_vel_body;   //当前线速度（机体）
        Eigen::Vector3d root_lin_vel_world;  //当前线速度（世界） 
        
        Vector18d X; //后验状态变量 第一次需要赋予初值
        Vector18d X_est;//先验状态变量
        Vector3d U;//输入
        Matrix18d A;//状态转移矩阵
        Matrix18x3d B;
        Vector28d Z;//观测
        Matrix28x18d H;//观测转移矩阵
        
        Matrix18d P;//后验误差协方差
        Matrix18d P_est;//先验误差协方差
        Matrix18d Q;//过程噪声协方差矩阵

        Matrix28d R;//观测噪声的协方差矩阵
        Matrix28d S; //增益系数k的分母 
        //为了方便计算 k的分子作为一部分 分母与后面的融合到一块 用llt求解加快求解速度
        Vector28d error_z;//z-h*x先验 观测误差
        Vector28d Serror_z; // S^-1*error_y
        Matrix28x18d SH;// S^-1*H



        //辅助
        Eigen::Matrix<double, 3, 3> eye3; // 单位矩阵3x3 identity
        Vector4d trust;//各个足端的置信度
};

}

#endif