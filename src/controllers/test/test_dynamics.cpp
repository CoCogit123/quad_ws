// 包含你定义的头文件
#include "Dynamics.h" 

#include <ros/ros.h>
#include <ros/package.h> // 用于获取功能包路径
#include <iostream>
#include <chrono> // 用于高精度计时
#include <random> // 用于生成随机测试数据



using namespace controllers;

/**
 * @brief 生成随机的 Robot_info 输入数据用于压力测试
 */
void generateRandomRobotState(Robot_info& robot) {
    // 随机数生成器
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_pos(-0.5, 0.5);
    std::uniform_real_distribution<> dis_angle(-1.0, 1.0);
    std::uniform_real_distribution<> dis_vel(-2.0, 2.0);

    // 1. 模拟传感器数据
    // 电机角度 (假设范围 -1 到 1 rad)
    for(int i=0; i<12; ++i) {
        robot.Pos_motor(i) = dis_angle(gen);
        robot.Vel_motor(i) = dis_vel(gen);
    }

    // 浮动基姿态 (随机四元数)
    Eigen::Vector4d rnd_quat;
    rnd_quat.setRandom();
    robot.quat_base = Eigen::Quaterniond(rnd_quat.normalized()); // 必须归一化
    robot.body_Rot_world = robot.quat_base.toRotationMatrix();

    // 浮动基位置 (世界系)
    robot.world_Pos_com << dis_pos(gen), dis_pos(gen), 0.3 + dis_pos(gen)*0.1;

    // IMU 数据 (机体系)
    robot.body_Omega << dis_vel(gen), dis_vel(gen), dis_vel(gen);
    robot.body_Acc << 0, 0, 9.81; // 模拟重力

    // 导出数据 (机体系速度 -> 世界系)
    // 假设 body_Vel_com 是已知的 (测试时可以反向算)
    robot.body_Vel_com << dis_vel(gen), dis_vel(gen), dis_vel(gen);
    robot.world_Vel_com = robot.body_Rot_world * robot.body_Vel_com;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamics_node");
    ros::NodeHandle nh;

    // 1. 获取 URDF 路径
    std::string urdf_pkg_dir = ros::package::getPath("robot_description");//urdf包路径
    std::string urdf_path = urdf_pkg_dir +  "/robot/urdf/robot.urdf";//urdf路径
    ROS_INFO("succes urdf_path: %s", urdf_path.c_str());

    // 2. 实例化 Dynamics 类
    controllers::Dynamics dynamics_solver(urdf_path);
    controllers::Robot_info robot_data;

    ROS_INFO("Dynamics Solver Initialized Successfully.");

    // ==========================================
    // Test 1: 单次计算准确性验证 (Sanity Check)
    // ==========================================
    ROS_INFO("--- Starting Sanity Check ---");
    
    // 设置一个已知的简单状态 (例如：趴下状态，所有关节为0)
    robot_data.Pos_motor.setZero();
    robot_data.Vel_motor.setZero();
    robot_data.quat_base.setIdentity(); // 无旋转
    robot_data.body_Rot_world.setIdentity();
    robot_data.world_Pos_com << 0, 0, 0.3; // 悬空 0.3m
    robot_data.body_Vel_com.setZero();
    robot_data.body_Omega.setZero();

    // 调用更新
    dynamics_solver.update(robot_data);

    // 验证输出
    std::cout << "[Check 1] Total Mass: " << robot_data.mass << " kg (Should be > 0)" << std::endl;
    std::cout << "[Check 2] Mass Matrix (M) Size: " << robot_data.M_q.rows() << "x" << robot_data.M_q.cols() << std::endl;
    std::cout << "[Check 3] Mass Matrix Symmetry Error: " 
              << (robot_data.M_q - robot_data.M_q.transpose()).norm() 
              << " (Should be close to 0)" << std::endl;
    
    std::cout << "[Check 4] Foot 1 (FL) Jacobian Size: " 
              << robot_data.J_foot[0].rows() << "x" << robot_data.J_foot[0].cols() << std::endl;

    std::cout << "[Check 5] World Foot Position (FL): " << robot_data.world_POS.col(0).transpose() << std::endl;
    
    // ==========================================
    // Test 2: 性能测试 (Benchmark)
    // ==========================================
    ROS_INFO("--- Starting Performance Benchmark ---");

    const int loop_count = 10000; // 运行 10000 次
    double total_time_us = 0.0;
    double max_time_us = 0.0;
    double min_time_us = 1e9;

    for (int i = 0; i < loop_count; ++i) {
        // 每次生成不同状态，防止编译器过度优化或缓存命中作弊
        generateRandomRobotState(robot_data);

        auto start = std::chrono::high_resolution_clock::now();
        
        // --- 核心测试函数 ---
        dynamics_solver.update(robot_data);
        // ------------------

        auto end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        total_time_us += duration;
        if (duration > max_time_us) max_time_us = duration;
        if (duration < min_time_us) min_time_us = duration;
    }

    double avg_time_us = total_time_us / loop_count;

    std::cout << "\n=== Benchmark Results (" << loop_count << " iterations) ===" << std::endl;
    std::cout << "Average Time: " << avg_time_us << " us (" << (1000000.0/avg_time_us) << " Hz)" << std::endl;
    std::cout << "Min Time:     " << min_time_us << " us" << std::endl;
    std::cout << "Max Time:     " << max_time_us << " us" << std::endl;
    
    // 评价标准：通常 MPC+WBC 循环需要控制在 1ms (1000us) 以内。
    // 其中动力学更新占用 50-200us 是比较理想的。
    if (avg_time_us < 300.0) {
        ROS_INFO("\033[32m[PASS] Performance is excellent for 1kHz control loop.\033[0m");
    } else if (avg_time_us < 800.0) {
        ROS_WARN("\033[33m[WARN] Performance is acceptable but tight for 1kHz control loop.\033[0m");
    } else {
        ROS_ERROR("\033[31m[FAIL] Calculation is too slow for real-time control!\033[0m");
    }

    return 0;
}