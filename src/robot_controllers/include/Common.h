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
    using Vector13d = Eigen::Matrix<double, 13, 1>;
    using Vector18d = Eigen::Matrix<double, 18, 1>;
    using Vector19d = Eigen::Matrix<double, 19, 1>; // Pinocchio中浮动基q是7维(4元数+3位置)
    using Vector4i = Eigen::Vector4i; //int
    using Vector28d = Eigen::Matrix<double, 28, 1>;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix3x4d = Eigen::Matrix<double, 3, 4>;
    using Matrix6x18d = Eigen::Matrix<double, 6, 18>;
    using Matrix3x18d = Eigen::Matrix<double, 3, 18>;
    using Matrix18d = Eigen::Matrix<double, 18, 18>;
    using Matrix18x3d = Eigen::Matrix<double, 18, 3>;
    using Matrix28d = Eigen::Matrix<double, 28, 28>;
    using Matrix28x18d = Eigen::Matrix<double, 28, 18>;
    using Matrix13x13d = Eigen::Matrix<double, 13, 13>;
    using Matrix13x12d = Eigen::Matrix<double, 13, 12>;
    /**
     * @brief 机器人信息结构体
     * 通用数据
    */
   struct Robot_info {
        // ==========================================
        // 1. 用户输入 (User Command) - 输入
        // ==========================================

        Vector3d body_Vel_des;   // 期望线速度-机体系       **********遥控器输入*************
        Vector3d world_Vel_des;  // 期望线速度-世界系       **********遥控器输入*************
        Vector3d body_omega_des; // 期望角速度-机体系       **********遥控器输入*************
        Vector3d world_omega_des;// 期望角速度-世界系       **********遥控器输入*************

        double z_des = 0.1;//期望高度（速度累计来）         **********遥控器输入*************
        Vector3d euler_des;//期望欧拉角（速度累计来）       **********遥控器输入*************

        double p_x_offest = -0.02;//足端默认支撑时的x偏移量（左上腿为标准 ）**********配置参数*************
        double p_y_offest = 0;//足端默认支撑时的y偏移量（左上腿为标准 ）    **********配置参数*************
        bool run_flag = false;//输入控制输出 急停！ 只控制电机输出 none模式run_flag = false或者是阻尼  
        bool safe_flag = true;//运行状态检测 第二层保证 出问题时进入阻尼模式
        // ==========================================
        // 2. 传感器数据 (Sensors) - 输入
        // ==========================================
        
        Vector12d Pos_motor;        // 12个电机的角度       **********传感器输入*************
        Vector12d Vel_motor;        // 12个电机的角速度     **********传感器输入*************
        
        Vector3d body_Acc;   // IMU的加速度 (机体系，通常需去除重力)     **********传感器输入*************
        Vector3d body_Omega; // IMU的角速度                             **********传感器输入*************          
        Vector3d euler;      // IMU的欧拉角 (Roll, Pitch, Yaw)          **********传感器输入*************
        Matrix3d body_Rot_world; // 旋转矩阵从body2world                **********传感器输入*************
        Eigen::Quaterniond quat_base;  // 四元数 (w,x,y,z) 记得归一化 注意Pinocchio通常是(x,y,z,w)需对应    **********传感器输入*************
        
        Vector3d world_Acc;   // 世界系的加速度 (估算值)        **********传感器输入*************

        // ==========================================
        // 3. 运动学解算 (Kinematics) - 输出
        // ==========================================

        Matrix3x4d body_POS; // 四条腿在机体坐标系下的位置  **********pinocchio*************
        Matrix3x4d world_POS;// 四条腿在世界坐标系下的位置  **********pinocchio*************
        
        Matrix3x4d body_VEL; // 四条腿在机体坐标系下的速度  **********pinocchio*************
        Matrix3x4d world_VEL;// 四条腿在世界坐标系下的速度  **********pinocchio*************
        
        Vector3d world_Pos_com; // 机体在世界坐标系下质心位置  **********作弊*************
        Vector3d world_Vel_com; // 机体在世界坐标系下质心速度   **********作弊*************
        Vector3d body_Vel_com;  // 机体在机体坐标系下质心速度   **********作弊*************

        Vector18d X_est;
        // ==========================================
        // 4. 动力学参数 (Dynamics Constants) - 静态/配置
        // ==========================================

        double mass;        // 机体总质量   **********pinocchio*************
        double mu = 0.3;          // 摩擦系数     **********配置参数*************
        
        // 结构偏移 (通常从URDF读取)
        double hx, hy, l1, l2, l3;      //  **********pinocchio*************

        Matrix3d body_INERTIA;  // 惯性矩阵（机体系）   **********pinocchio*************
        Matrix3d world_INERTIA; // 惯性矩阵（世界系）   **********pinocchio*************

        // ==========================================
        // 5. Pinocchio 动力学解算 (Dynamics Calculation) - 输出
        // ==========================================

        Matrix6x18d J_base;           // 雅可比矩阵 (浮动基) 关节空间 -> 线速度和角速度 **********pinocchio*************
        Matrix3x18d J_foot[4];        // 雅可比矩阵 (足端)   关节空间 -> 线速度 **********pinocchio*************

        Vector6d Jdt_qdt_base;    // 机身漂移加速度 是不是用不上 **********pinocchio*************
        Vector3d Jdt_qdt_foot[4];             // 足端漂移加速度 (只取线加速度) **********pinocchio*************

        Matrix18d M_q; // 广义质量矩阵 **********pinocchio*************
        Vector18d h_q_dq;                  // 非线性项 (科里奥利+离心+重力) **********pinocchio*************

        // ==========================================
        // 6. 最终电机控制
        // ==========================================

        Vector12d Pos_motor_cmd;        // 12个电机的控制角度
        Vector12d Vel_motor_cmd;        // 12个电机的控制角速度
        Vector12d Kp_motor;             // 12个电机的控制角速度
        Vector12d Kd_motor;             // 12个电机的控制角速度
        Vector12d Torque_motor;           // 12个电机的控制角速度

        // ==========================================
        // 构造函数初始化一些矩阵大小
        Robot_info() {
            body_Vel_des.setZero();
            world_Vel_des.setZero();
            body_omega_des.setZero();
            world_omega_des.setZero();
            euler_des.setZero();

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
        
        double run_time;//切换模式则重新计算运行时间
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
    
    /**
     * @brief 摆动相信息结构体
     * 
    */
    struct Swing_info {
        Matrix3x4d world_POS_mid_touch; //四条腿的对称点世界系
        Matrix3x4d world_POS_start_touch; //四条腿的起始点世界系
        Matrix3x4d world_POS_end_touch; //四条腿的落足点世界系
        Matrix3x4d link1_POS_foot; //四条腿的摆动位置跟踪点（相对于link1坐标系）
        Matrix3x4d link1_VEL_foot; //四条腿的摆动速度跟踪点（相对于link1坐标系）
        Matrix3x4d world_POS_foot; //四条腿的摆动位置跟踪点（相对于世界系）
        Matrix3x4d world_VEL_foot; //四条腿的摆动速度跟踪点（相对于世界系）
        Matrix3x4d leg_delta;//摆动腿关于对称点的增量（考虑了旋转的，旋转后的世界系）
        double swing_high = 0.1;//10cm

        double torque_kp = 100;
        double torque_kd = 1.0;
    };
}



#endif

