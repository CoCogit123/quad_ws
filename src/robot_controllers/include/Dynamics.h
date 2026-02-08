#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <memory>        // 必须包含，用于 std::unique_ptr
#include <string>
#include <vector>
#include <Eigen/Dense>   // 假设我们需要 Eigen 类型作为接口参数
#include "Common.h"      // 假设包含 Robot_info 和 Vector19d 等定义
#include "Utils.h"

namespace controllers {

// 前向声明：告诉编译器有个叫 DynamicsImpl 的类，但现在不需要知道它的细节
class Dynamics_Impl;

/**
 * @brief 动力学解算类 (Pimpl 模式)
 * @details 接口与实现完全分离，加速编译
 */
class Dynamics {
public:
    /**
     * @brief 构造函数
     * @param urdf_path URDF文件路径
     */
    Dynamics(const std::string& urdf_path);

    /**
     * @brief 析构函数
     * @note 必须在 .cpp 中实现，否则 unique_ptr 无法删除不完整的类型
     */
    ~Dynamics();

    /**
     * @brief 核心更新函数
     * @param robot 引用传入robot_info
     */
    void update(Robot_info& robot);

    /**
     * @brief 静态函数：姿态求足端相对位置
     * @note 虽然不依赖 Pinocchio，但移到 .cpp 可以减少头文件体积
     */
    static Vector3d posture_to_footpos(Robot_info& robot, int number);

    /**
     * @brief 静态函数：运动学逆解
     */
    static Vector3d inverse_kinematic(Vector3d pos, Robot_info& robot, int direction, int number);

private:
    // 指向实现的指针 (不透明指针)
    std::unique_ptr<Dynamics_Impl> pimpl_;
};

} // namespace controllers

#endif // DYNAMICS_H