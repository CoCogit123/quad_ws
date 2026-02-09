

// ==========================================
// 所有的 Pinocchio 头文件都放在 .cpp 里
// ==========================================
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include "Dynamics.h"

#include <iostream>
#include <cmath> // 用于 std::sqrt, std::atan 等

namespace controllers {

// ==========================================
// 真正存放数据的结构体 (Hidden Implementation)
// ==========================================
class Dynamics_Impl {
public:
    // 成员变量直接从原来的 private 搬过来
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

    // 构造逻辑
    Dynamics_Impl(const std::string& urdf_path) {
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

    void update_impl(Robot_info& robot) {
        // ... (此处完全复制原来的 update 代码) ...
        // ... 唯一的区别是这里直接访问成员变量 (如 model_), 不需要 this-> ...
        
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

            pinocchio::JointIndex id_hip   = model_.getJointId("leg2_joint1");   // 左上腿第1个坐标系
            pinocchio::JointIndex id_thigh = model_.getJointId("leg2_joint2"); // 左上腿第2个坐标系
            pinocchio::JointIndex id_calf  = model_.getJointId("leg2_joint3");  // 左上腿第3个坐标系
            pinocchio::FrameIndex id_foot = foot_indices_[2];
            Eigen::Vector3d pos_hx_hy = model_.jointPlacements[id_hip].translation();
            robot.hx = pos_hx_hy.x();
            robot.hy = pos_hx_hy.y();
            robot.l1 = model_.jointPlacements[id_thigh].translation().y();
            robot.l2 = -model_.jointPlacements[id_calf].translation().z();
            robot.l3 = -model_.frames[id_foot].placement.translation().z();

            init_ = 1;
        }
    }
};

// ==========================================
// Dynamics 类的接口实现 (Bridge)
// ==========================================

// 1. 构造函数：初始化 pimpl
Dynamics::Dynamics(const std::string& urdf_path) 
    : pimpl_(std::make_unique<Dynamics_Impl>(urdf_path)) {}

// 2. 析构函数：必须在这里写 default，
// 因为这里 DynamicsImpl 已经是一个完整的类型了，unique_ptr 知道怎么删它
Dynamics::~Dynamics() = default;

// 3. 核心更新函数：委托给 pimpl
void Dynamics::update(Robot_info& robot) {
    pimpl_->update_impl(robot);
}





    /**
     * @brief //姿态求足端相对位置
     * @param robot 引用传入robot_info，读取l1 l2 l3 hx hy 期望高度和欧拉角 默认偏移量并填入计算结果
     * @endif 导出3*1向量 足端相对于第一关节坐标系的坐标
     */
    Vector3d Dynamics::posture_to_footpos(Robot_info& robot,int number) 
    {
        Eigen::Vector3d Op_O;
        Op_O << 0, 0, -robot.z_des;
        //确认方向
        int dir_x,dir_y;
        switch(number)
        {
            case 0: dir_x = 1 ; dir_y = -1; //右上
                    break;
            case 1: dir_x = 1 ; dir_y = 1; //左上
                    break;
            case 2: dir_x = -1 ; dir_y = -1; //右下
                    break;
            case 3: dir_x = -1 ; dir_y = 1; //左下 
                    break;
        }
        //矢量加减
        Eigen::Vector3d O_B;
        O_B << robot.hx*dir_x+robot.p_x_offest, (robot.hy+robot.l1+robot.p_y_offest)*dir_y, 0.0;

        Eigen::Vector3d O_A;
        O_A << robot.hx*dir_x, dir_y*robot.hy, 0.0;

        Eigen::Matrix3d rot_mat;//旋转
        rot_mat = utils::euler_to_rot(robot.euler_des);
        
        Eigen::Vector3d Op_Ap;
        Op_Ap = rot_mat*O_A;

        Eigen::Vector3d Ap_B;

            Ap_B =  Op_O + O_B - Op_Ap;
        return Ap_B;
    }

    /**
     * @brief //运动学逆解
     * @param  pos 1.传入足端相对于第一关节坐标系的坐标 2.传入robot_info，读取l1 l2 l3 hx hy 3.方向默认后肘1 4.第几条腿
     * @endif 导出3*1向量 对于电机的角度
     */
    Vector3d Dynamics::inverse_kinematic(Vector3d pos, Robot_info& robot, int direction, int number) {
        //area_x 影响第一个关节 direction影响第二三个关节
        int area_x;
        switch(number)
        {
            case 0: area_x = 1 ; 
                    break;
            case 1: area_x = -1 ; 
                    break;
            case 2: area_x = 1 ; 
                    break;
            case 3: area_x = -1 ; 
                    break;
        }
        double x,y,z;
        double h,hu,hl;
        x = pos[0]; 
        y = pos[1]; 
        z = pos[2];
        h = robot.l1; hu = robot.l2; hl = robot.l3;
        /*******************第一个关节角度 只有唯一解 *****************************/
        double q0,r1,r2;

        double d_zoy;//yz指的是zoy平面
        d_zoy = std::sqrt( y*y + z*z );
        double l_zoy;
        l_zoy = std::sqrt( d_zoy*d_zoy -  h*h);

        r2 = -std::atan(y/z);
        //area_x不同 则offest不同
        r1 = -std::atan(h/l_zoy)*area_x;
        q0 = r2 - r1;

        // ROS_INFO("q0: %f,number: %d",q0/3.14*180,number);

        /*******************第三个关节角度 direction为1是后肘的  direction为-1是前肘*****************************/
        double q2;

        double S_xoz;
        S_xoz = std::sqrt( l_zoy*l_zoy + x*x );
        double n;
        n = ( S_xoz*S_xoz - hl*hl -hu*hu ) / ( 2 * hu );
        //前肘 后肘 相反 direction
        q2 = -std::acos( n / hl );

            // ROS_INFO("q2: %f,number: %d",q2/3.14*180,number);
            

        /*******************第二个关节角度 direction为1是后肘的  direction为-1是前肘*****************************/
        double q1;

        double alpha1_xoz,alpha2_xoz;
        alpha1_xoz = -std::atan( x / l_zoy );
        alpha2_xoz = std::acos( (hu + n) / S_xoz );
        //前肘 后肘 相反 direction
        q1 = ( alpha1_xoz + alpha2_xoz );
            // ROS_INFO("hu:%f",hu);
            // ROS_INFO("n:%f",n);
            // ROS_INFO("S_xoz:%f",S_xoz);
            // // ROS_INFO("l_zoy:%f",l_zoy);
            
            
            // ROS_INFO("alpha1_xoz: %f,number: %d",alpha1_xoz/3.14*180,number);
            // ROS_INFO("alpha2_xoz: %f,number: %d",alpha2_xoz/3.14*180,number);
            // ROS_INFO("q1: %f,number: %d",q1/3.14*180,number);

        Eigen::Vector3d result;
        result << q0, q1, q2;
        return result;
    }

} // namespace controllers