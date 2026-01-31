#ifndef DEBUG_H
#define DEBUG_H

// **************************
// 自定义头文件 
// **************************
#include "Common.h"
#include "Gait.h"
#include "Swing.h"
//自定义消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>
// **************************
// 其他头文件 
// **************************
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <sensor_msgs/Imu.h>//imu消息包

namespace controllers {
    /**
     * @brief 封装机器人信息打印调试函数
     * @param robot  Robot_info结构体
     * @param freq   打印频率 (Hz)
     */
    void Debug_robot_info(const Robot_info& robot, double freq) {
        static double last_print_time = 0;
        double current_time = ros::Time::now().toSec();

        // 频率控制逻辑：如果距离上次打印时间不足 1/freq 秒，则直接跳过
        if (current_time - last_print_time < (1.0 / freq)) {
            return;
        }
        last_print_time = current_time;

        printf("\n\033[1;36m==================== [robot_info] ====================\033[0m\n");
        printf("\033[1;32m==================== [target_commands] ====================\033[0m\n");
        // --- 1. 线速度指令 (Linear Velocity) ---
        printf("\033[1;33m[Linear Velocity Desired]\033[0m\n");
        printf("  Body  Vel (m/s): [%8.4f, %8.4f, %8.4f]\n", 
            robot.body_Vel_des.x(), robot.body_Vel_des.y(), robot.body_Vel_des.z());
        printf("  World Vel (m/s): [%8.4f, %8.4f, %8.4f]\n", 
            robot.world_Vel_des.x(), robot.world_Vel_des.y(), robot.world_Vel_des.z());
        // --- 2. 角速度指令 (Angular Velocity) ---
        printf("\033[1;33m[Angular Velocity Desired]\033[0m\n");
        printf("  Body  Omega (rad/s): [%8.4f, %8.4f, %8.4f]\n", 
            robot.body_omega_des.x(), robot.body_omega_des.y(), robot.body_omega_des.z());
        printf("  World Omega (rad/s): [%8.4f, %8.4f, %8.4f]\n", 
            robot.world_omega_des.x(), robot.world_omega_des.y(), robot.world_omega_des.z());
        // --- 3. 姿态与高度指令 (State Setpoints - Integrated) ---
        printf("\033[1;33m[Postural Setpoints (Integrated)]\033[0m\n");
        // 将欧拉角转为角度打印，更符合直觉
        printf("  Euler Des (deg): R:%7.2f, P:%7.2f, Y:%7.2f\n", 
            robot.euler_des.x() * 57.3, robot.euler_des.y() * 57.3, robot.euler_des.z() * 57.3);
        printf("  Height Des (m) : %8.4f  (Offset to Ground)\n", robot.z_des);

        // // --- Motor Data (按照四足机器人腿部逻辑排列) ---
        // printf("\033[1;33m[Motor States (Pos | Vel)]\033[0m\n");
        // for (int i = 0; i < 4; ++i) {
        //     printf("  Leg %d: [%8.4f, %8.4f, %8.4f] | [%8.4f, %8.4f, %8.4f]\n", i,
        //         robot.Pos_motor[i * 3], robot.Pos_motor[i * 3 + 1], robot.Pos_motor[i * 3 + 2],
        //         robot.Vel_motor[i * 3], robot.Vel_motor[i * 3 + 1], robot.Vel_motor[i * 3 + 2]);
        // }

        // // --- IMU & Orientation (包含计算后的 World Acc) ---
        // printf("\033[1;33m[IMU & Orientation]\033[0m\n");
        // printf("  Body Acc   (m/s^2): [%8.4f, %8.4f, %8.4f]\n", robot.body_Acc.x(), robot.body_Acc.y(), robot.body_Acc.z());
        // printf("  Body Omega (rad/s): [%8.4f, %8.4f, %8.4f]\n", robot.body_Omega.x(), robot.body_Omega.y(), robot.body_Omega.z());
        // printf("  Euler RPY    (deg): [%8.4f, %8.4f, %8.4f]\n", robot.euler.x() * 57.3, robot.euler.y() * 57.3, robot.euler.z() * 57.3);
        // printf("  Quat (w,x,y,z)    : [%8.4f, %8.4f, %8.4f, %8.4f]\n", 
        //     robot.quat_base.w(), robot.quat_base.x(), robot.quat_base.y(), robot.quat_base.z());
        // printf("  World Acc  (m/s^2): [%8.4f, %8.4f, %8.4f]\n", 
        //     robot.world_Acc.x(), robot.world_Acc.y(), robot.world_Acc.z());

        // // --- Kinematics ---
        // printf("\033[1;33m========= [Kinematics] =========\033[0m\n");
        // printf("\033[1;33m[Foot Positions (Body | World)]\033[0m\n");
        // printf("         |      FL      |      FR      |      RL      |      RR      |\n");
        // printf("  Body  X: %12.4f %12.4f %12.4f %12.4f\n", robot.body_POS(0,0), robot.body_POS(0,1), robot.body_POS(0,2), robot.body_POS(0,3));
        // printf("        Y: %12.4f %12.4f %12.4f %12.4f\n", robot.body_POS(1,0), robot.body_POS(1,1), robot.body_POS(1,2), robot.body_POS(1,3));
        // printf("        Z: %12.4f %12.4f %12.4f %12.4f\n", robot.body_POS(2,0), robot.body_POS(2,1), robot.body_POS(2,2), robot.body_POS(2,3));
        
        // printf("  World X: %12.4f %12.4f %12.4f %12.4f\n", robot.world_POS(0,0), robot.world_POS(0,1), robot.world_POS(0,2), robot.world_POS(0,3));
        // printf("        Y: %12.4f %12.4f %12.4f %12.4f\n", robot.world_POS(1,0), robot.world_POS(1,1), robot.world_POS(1,2), robot.world_POS(1,3));
        // printf("        Z: %12.4f %12.4f %12.4f %12.4f\n", robot.world_POS(2,0), robot.world_POS(2,1), robot.world_POS(2,2), robot.world_POS(2,3));
        // printf("\033[1;33m[com info (Body | World)]\033[0m\n");
        // printf("  World_Pos_Com   (m): [%8.4f, %8.4f, %8.4f]\n", robot.world_Pos_com[0], robot.world_Pos_com[1], robot.world_Pos_com[2]);
        // printf("  world_Vel_com   (m/s): [%8.4f, %8.4f, %8.4f]\n", robot.world_Vel_com[0], robot.world_Vel_com[1], robot.world_Vel_com[2]);
        // printf("  body_Vel_com   (m/s): [%8.4f, %8.4f, %8.4f]\n", robot.body_Vel_com[0], robot.body_Vel_com[1], robot.body_Vel_com[2]);

        // // --- Dynamics Constants ---
        // printf("\033[1;33m[Dynamics Constants]\033[0m\n");
        // printf("  Mass: %7.3f kg  |  Mu: %5.2f\n", robot.mass, robot.mu);
        // printf("  Structure: hx: %7.4f, hy: %7.4f\n", robot.hx, robot.hy);
        // printf("             l1: %7.4f, l2: %7.4f, l3: %7.4f\n", robot.l1, robot.l2, robot.l3);
        // printf("\033[1;33m[Inertia Matrices (Body | World)]\033[0m\n");
        // printf("  Body Inertia:             |  World Inertia:\n");
        // for (int i = 0; i < 3; ++i) {
        //     printf("  [%6.3f, %6.3f, %6.3f]  |  [%6.3f, %6.3f, %6.3f]\n",
        //         robot.body_INERTIA(i, 0), robot.body_INERTIA(i, 1), robot.body_INERTIA(i, 2),
        //         robot.world_INERTIA(i, 0), robot.world_INERTIA(i, 1), robot.world_INERTIA(i, 2));
        // }

        // printf("\033[1;33m========= [robot_dynamics_info] =========\033[0m\n");
        // // --- 1. 广义质量矩阵 M(q) 与 非线性项 h(q,dq) ---
        // printf("\033[1;33m[Mass Matrix & Nonlinear Terms]\033[0m\n");
        // printf("  M_q Norm: %10.4f | h_q_dq Norm: %10.4f\n", robot.M_q.norm(), robot.h_q_dq.norm());
        // // 打印 M 矩阵的前 6x6 部分（通常对应浮动基惯量）
        // printf("  M_q (Top-Left 6x6 diag): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
        //     robot.M_q(0,0), robot.M_q(1,1), robot.M_q(2,2), robot.M_q(3,3), robot.M_q(4,4), robot.M_q(5,5));
        // printf("  h_q_dq (First 6 elements): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
        //     robot.h_q_dq[0], robot.h_q_dq[1], robot.h_q_dq[2], robot.h_q_dq[3], robot.h_q_dq[4], robot.h_q_dq[5]);

        // // --- 2. 雅可比矩阵 J (浮动基 & 足端) ---
        // printf("\033[1;33m[Jacobian Matrices (Norms)]\033[0m\n");
        // printf("  J_base Norm: %10.4f\n", robot.J_base.norm());
        // printf("  J_foot Norms: FL:%8.3f | FR:%8.3f | RL:%8.3f | RR:%8.3f\n",
        //     robot.J_foot[0].norm(), robot.J_foot[1].norm(), robot.J_foot[2].norm(), robot.J_foot[3].norm());

        // // --- 3. 漂移加速度 (Jdot * qdot) ---
        // printf("\033[1;33m[Drift Accelerations (Jdot * qdot)]\033[0m\n");
        // printf("  Base Drift (Linear): [%8.4f, %8.4f, %8.4f]\n", 
        //     robot.Jdt_qdt_base[0], robot.Jdt_qdt_base[1], robot.Jdt_qdt_base[2]);
        // for(int i=0; i<4; ++i) {
        //     printf("  Foot %d Drift       : [%8.4f, %8.4f, %8.4f]\n", i,
        //         robot.Jdt_qdt_foot[i].x(), robot.Jdt_qdt_foot[i].y(), robot.Jdt_qdt_foot[i].z());
        // }

        printf("\033[1;36m======================================================\033[0m\n");
    }

    /**
     * @brief 打印步态逻辑相关信息
     * @param gait  Gait_info结构体
     * @param freq  打印频率 (Hz)
     */
    void Debug_gait_info(const Gait_info& gait, double freq) {
        static double last_print_time = 0;
        double current_time = ros::Time::now().toSec();
        if (current_time - last_print_time < (1.0 / freq)) return;
        last_print_time = current_time;

        printf("\n\033[1;35m==================== [gait_info] ====================\033[0m\n");

        // --- 1. 步态配置 (Static Params) ---
        printf("\033[1;33m[Configuration]\033[0m\n");
        printf("  Gait Mode: %-10d | Cycle Time: %.3fs\n", (int)gait.Gait_mode, gait.time_gait);
        printf("  Time Spec: Swing: %.3fs, Stand: %.3fs\n", gait.time_swing, gait.time_stand);
        printf("  Duty Cycle (Ratio) : [%.2f, %.2f, %.2f, %.2f]\n", 
            gait.Gait_ratio[0], gait.Gait_ratio[1], gait.Gait_ratio[2], gait.Gait_ratio[3]);
        printf("  Phase Offset       : [%.2f, %.2f, %.2f, %.2f]\n", 
            gait.Gait_offset[0], gait.Gait_offset[1], gait.Gait_offset[2], gait.Gait_offset[3]);

        // --- 2. 运行状态 (Runtime Variables) ---
        printf("\033[1;33m[Runtime Status]\033[0m\n");
        printf("  Global Time : %8.3fs | Cycles(N): %d\n", gait.run_time, gait.Gait_N);
        printf("  Global Phase: %8.4f  (0->1)\n", gait.time_gait_degree);

        // --- 3. 每条腿的具体状态 ---
        printf("\033[1;33m[Leg Status & Local Phase]\033[0m\n");
        printf("         |  State (1:S, 0:W) | Swing Phase | Stand Phase |\n");
        for(int i=0; i<4; ++i) {
            const char* leg_name = (i==0)?"FL":(i==1)?"FR":(i==2)?"RL":"RR";
            const char* state_str = (gait.Gait_state[i] == 1) ? "\033[1;32mSTANCE\033[0m" : "\033[1;34mSWING \033[0m";
            printf("  Leg %s:   [%s]      |   %8.4f  |   %8.4f  |\n", 
                leg_name, state_str, gait.Time_swing_degree[i], gait.Time_stand_degree[i]);
        }

        // --- 4. 标志位与信号 ---
        printf("\033[1;33m[Signals]\033[0m\n");
        const char* switch_str = (gait.Gait_flag == 1) ? "\033[1;31m!! CHANGING !!\033[0m" : "Stable";
        printf("  Flag: %s | Desired Mode: %d\n", switch_str, (int)gait.Gait_des);

        printf("\033[1;35m=====================================================\033[0m\n");
    }
}

#endif