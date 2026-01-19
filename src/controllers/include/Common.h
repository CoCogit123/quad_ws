#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>

namespace controllers {
    // --- 向量/矩阵常用定义 ---
    using Vector3d = Eigen::Vector3d;
    using Vector4d = Eigen::Vector4d;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using Vector18d = Eigen::Matrix<double, 18, 1>;
    using Vector19d = Eigen::Matrix<double, 19, 1>; // Pinocchio中浮动基q是7维(4元数+3位置)
    using Vector4i = Eigen::Vector4i; //int
    using Matrix3d = Eigen::Matrix3d;
    using Matrix3x4d = Eigen::Matrix<double, 3, 4>;
    using Matrix6x18d = Eigen::Matrix<double, 6, 18>;
    using Matrix3x18d = Eigen::Matrix<double, 3, 18>;
    using Matrix18d = Eigen::Matrix<double, 18, 18>;

    /**
     * @brief 机器人信息结构体
     * 通用数据
    */
   struct Robot_info {
        // ==========================================
        // 1. 用户输入 (User Command) - 输入
        // ==========================================

        Vector3d body_Vel_des;   // 期望线速度-机体系
        Vector3d world_Vel_des;  // 期望线速度-世界系
        Vector3d body_omega_des; // 期望角速度-机体系
        Vector3d world_omega_des;// 期望角速度-世界系

        // ==========================================
        // 2. 传感器数据 (Sensors) - 输入
        // ==========================================
        
        Vector12d Pos_motor;        // 12个电机的角度
        Vector12d Vel_motor;        // 12个电机的角速度
        
        Vector3d body_Acc;   // IMU的加速度 (机体系，通常需去除重力)
        Vector3d body_Omega; // IMU的角速度
        Vector3d euler;      // IMU的欧拉角 (Roll, Pitch, Yaw)
        Matrix3d body_Rot_world; // 旋转矩阵从body2world
        Eigen::Quaterniond quat_base;  // 四元数 (w,x,y,z) 记得归一化 注意Pinocchio通常是(x,y,z,w)需对应 
        
        Vector3d world_Acc;   // 世界系的加速度 (估算值)
        Vector3d world_Omega; // 世界系的角速度 (R * body_Omega)

        // ==========================================
        // 3. 运动学解算 (Kinematics) - 输出
        // ==========================================

        Matrix3x4d body_POS; // 四条腿在机体坐标系下的位置
        Matrix3x4d world_POS;// 四条腿在世界坐标系下的位置
        
        Matrix3x4d body_VEL; // 四条腿在机体坐标系下的速度
        Matrix3x4d world_VEL;// 四条腿在世界坐标系下的速度
        
        Vector3d world_Pos_com; // 机体在世界坐标系下质心位置
        Vector3d world_Vel_com; // 机体在世界坐标系下质心速度
        Vector3d body_Vel_com;  // 机体在机体坐标系下质心速度

        // ==========================================
        // 4. 动力学参数 (Dynamics Constants) - 静态/配置
        // ==========================================

        double mass;        // 机体总质量
        double mu;          // 摩擦系数
        
        // 结构偏移 (通常从URDF读取)
        double hx, hy, l1, l2, l3; 

        Matrix3d body_INERTIA;  // 惯性矩阵（机体系）
        Matrix3d world_INERTIA; // 惯性矩阵（世界系）

        // ==========================================
        // 5. Pinocchio 动力学解算 (Dynamics Calculation) - 输出
        // ==========================================

        Matrix6x18d J_base;           // 雅可比矩阵 (浮动基) 关节空间 -> 线速度和角速度
        Matrix3x18d J_foot[4];        // 雅可比矩阵 (足端)   关节空间 -> 线速度

        Vector6d Jdt_qdt_base;    // 机身漂移加速度 是不是用不上
        Vector3d Jdt_qdt_foot[4];             // 足端漂移加速度 (只取线加速度)

        Matrix18d M_q; // 广义质量矩阵
        Vector18d h_q_dq;                  // 非线性项 (科里奥利+离心+重力)

        // ==========================================
        // 构造函数初始化一些矩阵大小
        Robot_info() {
            J_base.setZero();
            for(int i=0; i<4; i++) {
                J_foot[i].setZero();
                Jdt_qdt_foot[i].setZero();
            }
            M_q.setZero();
            h_q_dq.setZero();
        }
    };
    /**
     * @brief 步态信息结构体
     * 
    */
    //步态类型枚举
    enum Gait_type {
        none,
        stand,
        walk,
        trot
    };
    struct Gait_info {
        // ==========================================
        // 1. 静态参数 (Configuration Parameters)
        // ==========================================
        
        Gait_type Gait_mode;             // 当前运行步态类型
        
        double time_gait;               // 一个周期时长 (秒)
        double time_swing;              //   摆动时长 (秒) 
        double time_stand;              //   支撑时长 (秒)
        
        Vector4d Gait_ratio;     //   当前运行步态占空比 (Duty Cycle)
        Vector4d Gait_offset;    //   当前运行步态相位偏移量 (Phase Offset)
        
        // 控制标志位
        // [0]: 是否切换步态 (1=切换, 0=保持)
        // [1]: 目标步态类型 (Gait_type)
        int Gait_flag;      // 修改信号
        Gait_type Gait_des; //期望修改模式

        // ==========================================
        // 2. 动态状态 (Runtime Variables)
        // ==========================================
        
        int Gait_N;                 // 运行过几个周期
        double time_gait_degree;        // 当前全局相位 [0, 1)
        
        Vector4i Gait_state;     // 每条腿的触地情况 (1=Stance, 0=Swing)
        
        Vector4d Time_swing_degree; // 当前摆动周期相位 [0, 1)
        Vector4d Time_stand_degree; // 当前支撑周期相位 [0, 1)

        // ==========================================
        // 构造函数 (初始化防止崩溃)
        // ==========================================
        Gait_info() {
            Gait_mode = none;
            
            time_gait = 0.5;
            time_swing = 0.2;
            time_stand = 0.3;
            
            Gait_ratio.setConstant(0.6);
            Gait_offset.setZero();
            Gait_flag = 0;
            Gait_des = none;
            
            Gait_N = 0;
            time_gait_degree = 0.0;
            Gait_state.setOnes(); // 默认全站立
            Time_swing_degree.setZero();
            Time_stand_degree.setZero();
        }
    };

}

#endif

