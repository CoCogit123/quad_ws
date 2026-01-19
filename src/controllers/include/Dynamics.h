#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>      // 用于非线性项 (h)
#include <pinocchio/algorithm/crba.hpp>      // 用于质量矩阵 (M)
#include <pinocchio/algorithm/frames.hpp>    // 用于Frame操作
#include <pinocchio/algorithm/center-of-mass.hpp> // 用于质心

#include "Common.h"

namespace controllers {


/**
 * @brief 动力学解算类
 * @details 封装 Pinocchio 库
 */
class Dynamics {
public:
    /**
     * @brief 构造函数，加载URDF模型
     * @param urdf_path URDF文件路径
     */
    Dynamics(const std::string& urdf_path) {
        // 1. 加载模型
        pinocchio::urdf::buildModel(urdf_path, root_joint_, model_);
        data_ = pinocchio::Data(model_);

        // 2. 获取关键Frame的ID (假设URDF中足端Frame名字如下)
        foot_names_ = {"leg1_link4", "leg2_link4", "leg3_link4", "leg4_link4"};
        for (const auto& name : foot_names_) {
            if (model_.existFrame(name)) {
                foot_indices_.push_back(model_.getFrameId(name));
            } else {
                std::cerr << "Error: Frame " << name << " not found in URDF!" << std::endl;
            }
        }
        
        // 3. 初始分配空间
        q_pin_ = Vector19d::Zero();
        v_pin_ = Vector18d::Zero();

    }

    /**
     * @brief 核心更新函数
     * @param robot 引用传入robot_info，读取输入并填入计算结果
     */
    void update(Robot_info& robot) {

        // =========================================================
        // Step 1: 数据转换 (Robot Info -> Pinocchio State)
        // =========================================================
        
        // 构建 q (配置向量): [Pos_base(3), Quat_base(4), Pos_joints(12)] 世界系
        // 注意：Pinocchio的四元数顺序通常为 [x, y, z, w]
        q_pin_.head<3>() = robot.world_Pos_com;
        q_pin_.segment<4>(3) = robot.quat_base.coeffs();
        q_pin_.tail<12>() = robot.Pos_motor;

        // 构建 v (速度向量): [Vel_linear_base(3), Vel_angular_base(3), Vel_joints(12)] 机体系
        // 注意：这里需要机体系下的线速度和机体下的角速度，具体取决于Pinocchio配置
        // 通常 JointModelFreeFlyer 的 v 前6维是 base 在其自身坐标系下的空间速度(spatial velocity)
        // 但为了通用，假设这里传入的是转换好的广义速度
        v_pin_.head<3>() = robot.body_Vel_com; 
        v_pin_.segment<3>(3) = robot.body_Omega;
        v_pin_.tail<12>() = robot.Vel_motor;

        // =========================================================
        // Step 2: 运行 Pinocchio 核心算法
        // =========================================================
        
        // 更新运动学和雅可比
        pinocchio::forwardKinematics(model_, data_, q_pin_, v_pin_);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_, q_pin_);
        
        // 更新动力学 (M, h)
        pinocchio::crba(model_, data_, q_pin_); // 更新 M
        pinocchio::nonLinearEffects(model_, data_, q_pin_, v_pin_); // 更新 h (nle)
        
        // 更新质心
        pinocchio::centerOfMass(model_, data_, q_pin_, v_pin_);

        // =========================================================
        // Step 3: 提取数据并填充到 robot_info (Pinocchio -> Robot Info)
        // =========================================================

        // 3.1 动力学矩阵 M(q) 和 h_q_dq
        robot.M_q = data_.M; // 加上电机转子惯量需要额外处理，这里仅为连杆
        robot.M_q.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>(); // 确保对称
        robot.h_q_dq = data_.nle;

        // 3.2 惯性矩阵 $$I_{\mathcal{O}}$$ & $$I_{\mathcal{B}}$$
        // 获取基座连杆(通常index=1)的惯性
        // 注意：data_.Ycrb[1] 包含复合刚体惯量，这里简化取基座模型惯量
        if(init_ == 1)
        {
            robot.world_INERTIA = robot.body_Rot_world * robot.body_INERTIA * robot.body_Rot_world.transpose();
        }
        
        // 3.3 足端运动学 (Pos, Vel)
        for(int i=0; i<4; i++) {
            long frame_id = foot_indices_[i];
            
            // 位置
            Eigen::Vector3d p_world = data_.oMf[frame_id].translation();
            robot.world_POS.col(i) = p_world;
            robot.body_POS.col(i) = robot.body_Rot_world.transpose() * (p_world - robot.world_Pos_com); // 简化的相对位置
            
            // 速度 (Pinocchio计算的是Frame spatial velocity)
            // getFrameVelocity 返回的是 Motion 向量 (linear, angular)
            pinocchio::Motion v_frame = pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
            robot.world_VEL.col(i) = v_frame.linear();
            robot.body_VEL.col(i) = robot.body_Rot_world.transpose() * v_frame.linear();
            
            // 3.4 足端雅可比 
            // Pinocchio 返回 6x18，我们取前3行(线速度)
            pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp_);
            robot.J_foot[i] = J_temp_.topRows<3>();

            // 3.5 足端漂移加速度 $$\dot{J} \dot{q}_{foot,i}$$
            // getFrameClassicalAcceleration 返回加速度 (linear 3d + angular 3d)
            pinocchio::Motion a_drift = pinocchio::getFrameClassicalAcceleration(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
            robot.Jdt_qdt_foot[i] = a_drift.linear();
        }

        // 3.6 基座雅可比 $$J_{base}$$
        // 获取基座关节(root)的雅可比，通常设 JointIndex = 1 (视URDF而定，FreeFlyer通常是1)
        // 注意：需要将局部雅可比转换到世界系对齐
        pinocchio::getJointJacobian(model_, data_, root_joint_indices_, pinocchio::LOCAL_WORLD_ALIGNED, robot.J_base);
        
        // 3.7 基座漂移加速度 $$\dot{J} \dot{q}_{base}$$
        // 这是一个比较特殊的计算，通常可以用整机的加速度减去 M逆*tau 得到，
        // 或者简单地认为如果我们要控制浮动基，这部分包含在 h(q,dq) 中对应的项里。
        // 但如果作为单独项：
        // robot.Jdt_qdt_base = ... (在WBC中通常结合h项一起处理，或通过 getFrameAcceleration 获得)


        // =========================================================
        // Step init: 提取不变的数据并填充到 Robot_info
        // =========================================================
        if(init_ == 0)
        {
            robot.mass = data_.mass[0]; // 总质量
            robot.body_INERTIA = model_.inertias[1].inertia().matrix();
            init_ = 1;
        }
    }

private:
    int init_ = 0;
    pinocchio::Model model_;
    pinocchio::Data data_;
    
    Vector19d q_pin_;
    Vector18d v_pin_;
    
    pinocchio::JointModelFreeFlyer root_joint_;// 定义一个自由浮动关节模型
    pinocchio::JointIndex root_joint_indices_;//浮动基的
    std::vector<std::string> foot_names_;
    std::vector<pinocchio::FrameIndex> foot_indices_;

    // 临时变量，避免重复内存分配
    Matrix6x18d J_temp_;
};

}

#endif // PINOCCHIO_ROBOT_SYSTEM_H