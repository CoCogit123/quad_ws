#include "Dynamics.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>      // 用于非线性项 (h)
#include <pinocchio/algorithm/crba.hpp>      // 用于质量矩阵 (M)
#include <pinocchio/algorithm/frames.hpp>    // 用于Frame操作
#include <pinocchio/algorithm/center-of-mass.hpp> // 用于质心

#include <iostream>

namespace controllers {

void Dynamics::init_model(const std::string &urdf_path) {
    // 1. 加载 URDF 模型，指定根节点为浮动基座 (FreeFlyer)
    pinocchio::JointModelFreeFlyer root_joint;// 定义一个自由浮动关节模型
    try {
        pinocchio::urdf::buildModel(urdf_path, root_joint, model_);
    } catch (const std::exception& e) {
        std::cerr << "[Dynamics Error] Failed to load URDF: " << e.what() << std::endl;
        return;
    }
    // 2. 初始化 Data 对象
    data_ = pinocchio::Data(model_);

    // 3. 初始化辅助向量大小
    q_pin_ = Eigen::VectorXd::Zero(model_.nq); // nq = 19 (7 + 12)
    v_pin_ = Eigen::VectorXd::Zero(model_.nv); // nv = 18 (6 + 12)
    a_zero_ = Eigen::VectorXd::Zero(model_.nv); // 全0加速度

    // 4. 获取足端 Frame 索引 和body的索引(需要根据你的URDF实际名称修改!)
    body_index = model_.getJointId("root_joint");

    //足端
    std::vector<std::string> foot_names = {"leg1_link4", "leg2_link4", "leg3_link4", "leg4_link4"};
    for(int i=0; i<4; i++) {
        if(model_.existFrame(foot_names[i])) {
            foot_index[i] = model_.getFrameId(foot_names[i]);
        } else {
            std::cerr << "[Dynamics Warning] Frame not found: " << foot_names[i] << std::endl;
            foot_index[i] = 0; 
        }
    }

    std::cout << "[Dynamics] Model initialized successfully." << std::endl;

    
}

void Dynamics::update_dynamics(Robot_info &robot_data) {
    // ==========================================
    // 1. 数据转换 (Data Mapping)
    // 将 Robot_info 的数据填入 Pinocchio 需要的 q 和 v 向量
    // ==========================================
    
    // 位置 q: [Pos(3), Quat(4), Joints(12)]
    // 注意: Pinocchio 四元数顺序为 [x, y, z, w], Eigen 是 [w, x, y, z]
    // 必须使用 coeffs() 获取 [x,y,z,w]
    q_pin_.head<3>() = robot_data.base_pos;
    q_pin_.segment<4>(3) = robot_data.imu_quat.coeffs(); 
    q_pin_.tail(12) = robot_data.motor_q;

    // 速度 v: [LinVel(3), AngVel(3), JointsVel(12)]
    // 注意: 这里通常使用世界系下的线速度和机体系下的角速度(或者是LocalWorldAligned)
    // 具体取决于 StateEstimator 的输出定义，这里假设 base_vel 是世界系线速度
    v_pin_.head<3>() = robot_data.base_vel; // 世界系线速度
    v_pin_.segment<3>(3) = robot_data.imu_gyro; // 机体系角速度
    v_pin_.tail(12) = robot_data.motor_dq;

    // ==========================================
    // 2. 核心动力学计算 (Core Computation)
    // ==========================================
    
    // [关键技巧]: 传入 acc=0 调用 forwardKinematics
    // 作用1: 更新所有关节和Frame的位置、速度
    // 作用2: 计算仅由速度产生的加速度项 (也就是科氏力漂移项 dJ*dq)
    pinocchio::forwardKinematics(model_, data_, q_pin_, v_pin_, a_zero_);
    
    // 更新 Frame 坐标系 (必须在 forwardKinematics 之后)
    pinocchio::updateFramePlacements(model_, data_);
    
    // 计算 质量矩阵 M (结果存储在 data_.M 的上三角)
    pinocchio::crba(model_, data_, q_pin_);
    
    // 计算 非线性项 h = C*dq + g (结果存储在 data_.nle)
    pinocchio::nonLinearEffects(model_, data_, q_pin_, v_pin_);
    
    // 计算 雅可比矩阵 (结果存储在 data_.J)
    pinocchio::computeJointJacobians(model_, data_, q_pin_);
    
    // 计算 质心位置和速度 (结果存储在 data_.com[0], data_.vcom[0])
    pinocchio::centerOfMass(model_, data_, q_pin_, v_pin_);

    // ==========================================
    // 3. 填充结果回结构体 (Fill Back)
    // ==========================================

    // --- MPC 参数填充 ---
    robot_data.com_pos = data_.com[0]; // 世界系 CoM 位置
    robot_data.com_vel = data_.vcom[0]; // 世界系 CoM 速度
    
    // 获取整机惯量 (Pinocchio 计算的是相对于 CoM 的惯量)
    // 这里的 ig/Ycrb是 spatial::Inertia 类型，提取 matrix() 得到 3x3 矩阵
    robot_data.composite_inertia = data_.Ycrb[0].inertia().matrix(); 

    // --- WBC 参数填充 ---
    
    // 填充 M (对称化处理，因为 CRBA 只算上三角)
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    robot_data.wbc_M = data_.M; // [18x18]
    
    // 填充 h
    robot_data.wbc_h = data_.nle; // [18]

    // 遍历足端填充 J 和 dJdq
    for(int i=0; i<4; i++) {
        int idx = foot_index[i];

        // 1. 足端位置 (世界系)
        robot_data.foot_pos_world[i] = data_.oMf[idx].translation();

        // 2. 雅可比矩阵
        // 使用 LOCAL_WORLD_ALIGNED (原点在足端，方向与世界对齐) 是控制最常用的
        Eigen::MatrixXd J_temp(6, model_.nv); 
        J_temp.setZero();
        pinocchio::getFrameJacobian(model_, data_, idx, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);
        
        // WBC 通常只需要 3x18 (线性部分)，忽略转动约束(除非是点接触)
        robot_data.wbc_J[i] = J_temp.topRows(3); 

        // 3. 雅可比漂移项 (dJ * dq)
        // 因为我们传入了 acc=0，这里的 ClassicalAcceleration 就是 Coriolis 加速度
        pinocchio::Motion a_drift = pinocchio::getFrameClassicalAcceleration(model_, data_, idx, pinocchio::LOCAL_WORLD_ALIGNED);
        robot_data.wbc_dJdq[i] = a_drift.linear(); // [3x1]
    }
}

}