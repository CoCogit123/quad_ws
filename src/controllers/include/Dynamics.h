#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>

namespace controllers {
// --- 定义机器人数据结构体 (DataBus) ---
struct Robot_info {
    // ==========================================
    // --- 传感器数据 (输入) ---
    // ==========================================
    // IMU 数据
    Eigen::Quaterniond imu_quat;    // [w, x, y, z] 机体(Body)相对于世界(World)的姿态
    Eigen::Vector3d imu_gyro;       // [rx, ry, rz] 机体坐标系下的角速度 (rad/s)
    Eigen::Vector3d imu_acc;        // [ax, ay, az] 机体坐标系下的线性加速度 (m/s^2)
    // 电机数据 (12个关节)
    // 顺序通常为: leg1_link1 leg1_link2 leg1_link3 leg2_link1 ... leg4_link4
    Eigen::VectorXd motor_q;        // [12] 关节角度 (rad)
    Eigen::VectorXd motor_dq;       // [12] 关节角速度 (rad/s)
    // ==========================================
    // --- 估计数据 (输入) ---
    // 由状态估计器(Estimator)计算得出
    // ==========================================
    Eigen::Vector3d base_pos;       // [x, y, z] 机体在世界坐标系下的位置 (m)
    Eigen::Vector3d base_vel;       // [vx, vy, vz] 机体在世界坐标系下的线速度 (m/s)
    // ==========================================
    // --- 状态机 (输入/输出) ---
    // 用于逻辑判断
    // ==========================================
    Eigen::Vector4i contact_state;  // [4] 足端接触状态 (1: 接触, 0: 摆动)
    double gait_phase[4];           // [4] 步态相位 (0~1)
    // ==========================================
    // --- MPC 需要的参数 ---
    // ==========================================
    Eigen::Vector3d com_pos;        // [x, y, z] 整机质心在世界坐标系下的位置
    Eigen::Vector3d com_vel;        // [vx, vy, vz] 整机质心在世界坐标系下的速度
    Eigen::Vector3d foot_pos_world[4]; // [4] 足端在世界坐标系下的位置 (用于计算 r = p_foot - p_com)
    Eigen::Matrix3d composite_inertia;// [3x3] 整机复合惯性张量 (世界系或机体系，通常MPC需转换到机体系)
    // ==========================================
    // --- WBC 需要的参数 ---
    // ==========================================
    // 动力学方程: M * ddq + h = J^T * F + tau
    // 任务空间: ddx = J * ddq + dJdq
    Eigen::MatrixXd wbc_M;          // [18x18] 广义质量矩阵 (Mass Matrix)
    Eigen::VectorXd wbc_h;          // [18] 非线性项 (包含科氏力 Coriolis + 重力 Gravity)
    Eigen::MatrixXd wbc_J[4];       // [4个 3x18] 足端雅可比矩阵 (世界对齐坐标系 LocalWorldAligned)
    Eigen::Vector3d wbc_dJdq[4];    // [4个 3x1] 雅可比漂移项 (Coriolis bias, dJ * dq) - 替代计算巨大的 dJ 矩阵
};

// --- 动力学计算类 ---
class Dynamics {
public:
    /**
     * @brief 初始化模型函数
     * @param urdf_path 机器人URDF文件的绝对路径
     */
    void init_model(const std::string &urdf_path);

    /**
     * @brief 数据更新函数
     * 传入结构体，根据结构体中的“传感器数据”和“估计数据”，
     * 重新计算出MPC和WBC需要的参数，并直接写回结构体中。
     * @param robot_data 机器人全量信息结构体的引用
     */
    void update_dynamics(Robot_info &robot_data);

private:
    // --- Pinocchio 核心变量 ---
    pinocchio::Model model_;        // 机器人运动学/动力学模型 (常用于存储物理参数)
    pinocchio::Data data_;          // 机器人计算过程数据 (存储计算结果，如M, J)
    
    // --- 辅助变量 ---
    Eigen::VectorXd q_pin_;         // [19] Pinocchio 格式的广义坐标 [pos(3), quat(4), joints(12)]
    Eigen::VectorXd v_pin_;         // [18] Pinocchio 格式的广义速度 [lin_vel(3), ang_vel(3), joints_vel(12)]
    Eigen::VectorXd a_zero_;        // [18] 零加速度向量，用于提取漂移项
    
    //关节索引
    pinocchio::JointIndex body_index;//浮动基的
        //浮动基足端索引
    int foot_index[4];

};

}

#endif // PINOCCHIO_ROBOT_SYSTEM_H